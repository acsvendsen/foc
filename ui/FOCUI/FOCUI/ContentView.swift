import SwiftUI
import Combine

enum TelemetryMetric: String, CaseIterable, Identifiable {
    case trackingError = "Tracking Error"
    case iqMeasured = "Iq Measured"
    case estimatedMotorTorque = "Est. Motor Torque"
    case velocity = "Velocity"
    case position = "Position"

    var id: String { rawValue }

    var unit: String {
        switch self {
        case .trackingError, .position:
            return "turns"
        case .iqMeasured:
            return "A"
        case .estimatedMotorTorque:
            return "Nm"
        case .velocity:
            return "turns/s"
        }
    }

    func value(for sample: TelemetrySample) -> Double? {
        switch self {
        case .trackingError:
            return sample.trackingError
        case .iqMeasured:
            return sample.iqMeas
        case .estimatedMotorTorque:
            return sample.estimatedMotorTorqueNm
        case .velocity:
            return sample.velEst
        case .position:
            return sample.posEst
        }
    }
}

@MainActor
final class LiveMonitorModel: ObservableObject {
    private static let maxTelemetrySamples = 60

    @Published var snapshot: BackendSnapshot?
    @Published var capabilities: BackendCapabilities?
    @Published var device: BackendDevice?
    @Published var telemetrySamples: [TelemetrySample] = []

    func merge(from result: BackendResponse) {
        if let snapshot = result.snapshot {
            self.snapshot = snapshot
        }
        if let capabilities = result.capabilities {
            self.capabilities = capabilities
        }
        if let device = result.device {
            self.device = device
        }
    }

    func appendTelemetrySample(from result: BackendResponse, fallbackTc: Double?) {
        guard let snap = result.snapshot,
              let posEst = snap.pos_est,
              let velEst = snap.vel_est,
              let iqMeas = snap.Iq_meas,
              let inputPos = snap.input_pos
        else { return }
        let tc = snap.tc ?? snapshot?.tc ?? fallbackTc
        let now = Date()
        appendTelemetrySample(
            TelemetrySample(
                id: now.timeIntervalSinceReferenceDate,
                timestampS: now.timeIntervalSince1970,
                posEst: posEst,
                velEst: velEst,
                iqMeas: iqMeas,
                inputPos: inputPos,
                estimatedMotorTorqueNm: tc.map { iqMeas * $0 }
            )
        )
    }

    func appendGraphTelemetrySample(_ payload: GraphTelemetrySamplePayload) {
        guard let posEst = payload.pos_est,
              let velEst = payload.vel_est,
              let iqMeas = payload.Iq_meas,
              let inputPos = payload.input_pos
        else { return }
        let timestampS = payload.timestamp_s ?? Date().timeIntervalSince1970
        appendTelemetrySample(
            TelemetrySample(
                id: timestampS,
                timestampS: timestampS,
                posEst: posEst,
                velEst: velEst,
                iqMeas: iqMeas,
                inputPos: inputPos,
                estimatedMotorTorqueNm: payload.estimated_motor_torque_nm
            )
        )
    }

    func clearTelemetryHistory() {
        telemetrySamples.removeAll(keepingCapacity: false)
    }

    private func appendTelemetrySample(_ sample: TelemetrySample) {
        telemetrySamples.append(sample)
        if telemetrySamples.count > Self.maxTelemetrySamples {
            telemetrySamples.removeFirst(telemetrySamples.count - Self.maxTelemetrySamples)
        }
    }
}

@MainActor
final class OperatorConsoleViewModel: ObservableObject {
    static let sliderPresetAngles: [Double] = [0, 30, 45, 60, 90, 120, 180, 270, 360]

    @Published var repoRoot: String
    @Published var axisIndex: Int = 0
    @Published var kvEstimate: String = "140"
    @Published var lineLineROhm: String = "0.30"
    @Published var settleSeconds: String = "0.15"
    @Published var debugMode: Bool = false

    @Published var moveForm = MoveFormState()
    @Published var profileEditor = ProfileEditorFormState()
    @Published var sliderFollow = SliderFollowState()
    @Published var response: BackendResponse?
    @Published var telemetryAutoRefresh = false
    @Published var telemetryMetric: TelemetryMetric = .trackingError
    @Published var blockingActionInFlight = false
    @Published var lastClientError: String?

    private let backend = BackendClient()
    let liveMonitor = LiveMonitorModel()
    private var sliderDebounceTask: Task<Void, Never>?
    private var sliderQueuedAngle: Double?
    private var sliderCommandActive = false
    private var telemetryRequestActive = false
    private var backendEventTask: Task<Void, Never>?
    private var initialRefreshDone = false
    private var streamingStarted = false

    init() {
        self.repoRoot = backend.detectRepoRoot()
    }

    private func requestContext() -> BackendClient.RequestContext {
        BackendClient.RequestContext(
            repoRoot: repoRoot,
            axisIndex: axisIndex,
            kvEstimate: kvEstimate,
            lineLineROhm: lineLineROhm,
            settleSeconds: settleSeconds,
            debug: debugMode
        )
    }

    var isBusy: Bool { blockingActionInFlight }
    var capabilities: BackendCapabilities? { liveMonitor.capabilities ?? response?.capabilities }
    var diagnosis: BackendDiagnosis? { response?.diagnosis }
    var snapshot: BackendSnapshot? { liveMonitor.snapshot ?? response?.snapshot }
    var profiles: [String] { response?.available_profiles ?? [] }
    var profileDetails: [BackendProfileDetail] { response?.available_profile_details ?? [] }
    var hasAbsoluteZeroAnchor: Bool { !moveForm.zeroTurnsMotor.trimmingCharacters(in: .whitespaces).isEmpty }

    func ensureInitialRefresh() async {
        guard !initialRefreshDone else { return }
        initialRefreshDone = true
        await refreshStatus()
    }

    func refreshStatus() async { await run(action: "status") }
    func runDiagnose() async { await run(action: "diagnose") }
    func runFactSheet() async { await run(action: "fact-sheet") }
    func startup() async { await run(action: "startup", arguments: ["--timeout-s", "30"]) }
    func idle() async { await run(action: "idle") }
    func clearErrors() async { await run(action: "clear-errors") }

    func captureZeroHere() {
        guard let pos = snapshot?.pos_est else { return }
        moveForm.zeroTurnsMotor = String(format: "%.6f", pos)
        moveForm.relativeToCurrent = false
    }

    func sliderFollowToggled(_ enabled: Bool) async {
        if !enabled {
            sliderDebounceTask?.cancel()
            sliderQueuedAngle = nil
            return
        }
        if moveForm.releaseAfterMove {
            sliderFollow.liveEnabled = false
            sliderDebounceTask?.cancel()
            sliderQueuedAngle = nil
            lastClientError = "Live follow cannot release to IDLE after each update. Use presets or 'Send Slider Target Now' with release mode."
            return
        }
        queueSliderTarget(angle: sliderFollow.angleDeg)
    }

    func sliderAngleDidChange() {
        guard sliderFollow.liveEnabled else { return }
        queueSliderTarget(angle: sliderFollow.angleDeg)
    }

    func sendSliderTargetNow() async {
        if moveForm.releaseAfterMove {
            await issueSliderOneShotMove(angle: sliderFollow.angleDeg)
        } else {
            await issueSliderTarget(angle: sliderFollow.angleDeg)
        }
    }

    func selectSliderPreset(_ angle: Double) async {
        sliderFollow.angleDeg = angle
        if moveForm.releaseAfterMove {
            await issueSliderOneShotMove(angle: angle)
        } else {
            await issueSliderTarget(angle: angle)
        }
    }

    func releaseAfterMoveChanged(_ enabled: Bool) {
        if enabled {
            sliderFollow.liveEnabled = false
            sliderDebounceTask?.cancel()
            sliderQueuedAngle = nil
        }
    }

    func telemetryAutoRefreshChanged(_ enabled: Bool) {
        Task {
            if enabled {
                await ensureStreaming()
            } else {
                liveMonitor.clearTelemetryHistory()
                await disableStreamingIfAllowed()
            }
        }
    }

    func pollTelemetryOnce() async {
        guard !telemetryRequestActive else { return }
        telemetryRequestActive = true
        defer { telemetryRequestActive = false }
        do {
            let context = BackendClient.RequestContext(
                repoRoot: repoRoot,
                axisIndex: axisIndex,
                kvEstimate: kvEstimate,
                lineLineROhm: lineLineROhm,
                settleSeconds: settleSeconds,
                debug: debugMode
            )
            let result = try await backend.run(action: "telemetry", arguments: [], context: context)
            mergeProfilesIfNeeded(from: result)
            mergeLiveStatusIfNeeded(from: result)
            liveMonitor.appendTelemetrySample(from: result, fallbackTc: response?.snapshot?.tc)
        } catch {
            lastClientError = error.localizedDescription
            telemetryAutoRefresh = false
        }
    }

    func moveContinuous() async {
        await ensureStreaming()
        var args = [
            "--angle-deg", moveForm.angleDeg,
            "--angle-space", moveForm.angleSpace,
            "--profile-name", moveForm.profileName,
            "--gear-ratio", moveForm.gearRatio,
        ]
        if moveForm.relativeToCurrent {
            args.append("--relative-to-current")
        } else if !moveForm.zeroTurnsMotor.trimmingCharacters(in: .whitespaces).isEmpty {
            args.append(contentsOf: ["--zero-turns-motor", moveForm.zeroTurnsMotor])
        }
        if !moveForm.timeoutSeconds.trimmingCharacters(in: .whitespaces).isEmpty {
            args.append(contentsOf: ["--timeout-s", moveForm.timeoutSeconds])
        }
        if moveForm.releaseAfterMove {
            args.append("--release-after-move")
        }
        await run(action: "move-continuous-async", arguments: args, countsAsBlocking: false)
    }

    func loadProfileEditor(name: String? = nil) async {
        let profileName = (name ?? moveForm.profileName).trimmingCharacters(in: .whitespacesAndNewlines)
        guard !profileName.isEmpty else { return }
        await run(
            action: "profile-config",
            arguments: ["--profile-name", profileName],
            countsAsBlocking: false,
            storePrimaryResponse: true,
            storeTelemetryResponse: false
        )
    }

    func saveProfileEditor() async {
        do {
            let payload = try profileEditor.jsonPayload()
            await run(
                action: "save-profile",
                arguments: ["--profile-json", payload],
                countsAsBlocking: true,
                storePrimaryResponse: true,
                storeTelemetryResponse: false
            )
        } catch {
            lastClientError = error.localizedDescription
        }
    }

    private func run(
        action: String,
        arguments: [String] = [],
        countsAsBlocking: Bool = true,
        storePrimaryResponse: Bool = true,
        storeTelemetryResponse: Bool = true
    ) async {
        if countsAsBlocking {
            blockingActionInFlight = true
        }
        lastClientError = nil
        defer {
            if countsAsBlocking {
                blockingActionInFlight = false
            }
        }
        do {
            let result = try await backend.run(action: action, arguments: arguments, context: requestContext())
            if storePrimaryResponse {
                response = result
            }
            if storeTelemetryResponse {
                mergeLiveStatusIfNeeded(from: result)
            }
            mergeProfilesIfNeeded(from: result)
            mergeProfileEditorIfNeeded(from: result)
            liveMonitor.appendTelemetrySample(from: result, fallbackTc: response?.snapshot?.tc)
        } catch {
            lastClientError = error.localizedDescription
        }
    }

    private func ensureStreaming() async {
        guard !streamingStarted else { return }
        do {
            let stream = try await backend.ensureEventStream(context: requestContext(), intervalMs: 40)
            streamingStarted = true
            backendEventTask?.cancel()
            backendEventTask = Task { @MainActor [weak self] in
                guard let self else { return }
                for await event in stream {
                    self.handleStreamEvent(event)
                }
            }
        } catch {
            lastClientError = error.localizedDescription
            streamingStarted = false
        }
    }

    private func disableStreamingIfAllowed(force: Bool = false) async {
        let motionActive = liveMonitor.capabilities?.motion_active == true
        if !force && (telemetryAutoRefresh || motionActive) {
            return
        }
        guard streamingStarted else { return }
        do {
            try await backend.disableEventStream(context: requestContext())
        } catch {
            lastClientError = error.localizedDescription
        }
        backendEventTask?.cancel()
        backendEventTask = nil
        streamingStarted = false
    }

    private func handleStreamEvent(_ event: BackendResponse) {
        switch event.action {
        case "stream-graph":
            if telemetryAutoRefresh, let sample = event.graph_sample {
                liveMonitor.appendGraphTelemetrySample(sample)
            }
        case "stream-motion-status":
            mergeProfilesIfNeeded(from: event)
            mergeLiveStatusIfNeeded(from: event)
            response = event
            if telemetryAutoRefresh {
                liveMonitor.appendTelemetrySample(from: event, fallbackTc: response?.snapshot?.tc)
            }
            if event.capabilities?.motion_active == false {
                Task { await disableStreamingIfAllowed() }
            }
        default:
            mergeProfilesIfNeeded(from: event)
            mergeLiveStatusIfNeeded(from: event)
            break
        }
    }

    private func mergeProfilesIfNeeded(from result: BackendResponse) {
        if moveForm.profileName.isEmpty, let first = result.available_profiles?.first {
            moveForm.profileName = first
        }
        let available = result.available_profiles ?? profiles
        if available.contains(moveForm.profileName) == false, let first = available.first {
            moveForm.profileName = first
        }
    }

    private func mergeProfileEditorIfNeeded(from result: BackendResponse) {
        guard let editor = result.profile_editor else { return }
        profileEditor = ProfileEditorFormState(editor: editor)
        if moveForm.profileName != editor.name {
            moveForm.profileName = editor.name
        }
    }

    private func mergeLiveStatusIfNeeded(from result: BackendResponse) {
        liveMonitor.merge(from: result)
    }

    private func queueSliderTarget(angle: Double) {
        sliderQueuedAngle = angle
        sliderDebounceTask?.cancel()
        sliderDebounceTask = Task { @MainActor in
            try? await Task.sleep(nanoseconds: 30_000_000)
            await flushSliderQueue()
        }
    }

    private func flushSliderQueue() async {
        guard !sliderCommandActive else { return }
        sliderCommandActive = true
        defer { sliderCommandActive = false }
        while sliderFollow.liveEnabled, let angle = sliderQueuedAngle {
            sliderQueuedAngle = nil
            await issueSliderTarget(angle: angle)
        }
    }

    private func issueSliderTarget(angle: Double) async {
        guard hasAbsoluteZeroAnchor else {
            sliderFollow.liveEnabled = false
            sliderDebounceTask?.cancel()
            sliderQueuedAngle = nil
            lastClientError = "Capture current position as zero before using the live angle slider."
            return
        }
        let args = [
            "--angle-deg", String(format: "%.3f", angle),
            "--angle-space", "gearbox_output",
            "--profile-name", moveForm.profileName,
            "--gear-ratio", moveForm.gearRatio,
            "--zero-turns-motor", moveForm.zeroTurnsMotor,
        ]
        await run(
            action: "follow-angle",
            arguments: args,
            countsAsBlocking: false,
            storePrimaryResponse: false,
            storeTelemetryResponse: true
        )
    }

    private func issueSliderOneShotMove(angle: Double) async {
        guard hasAbsoluteZeroAnchor else {
            lastClientError = "Capture current position as zero before using slider presets or one-shot slider moves."
            return
        }
        await ensureStreaming()
        var args = [
            "--angle-deg", String(format: "%.3f", angle),
            "--angle-space", "gearbox_output",
            "--profile-name", moveForm.profileName,
            "--gear-ratio", moveForm.gearRatio,
            "--zero-turns-motor", moveForm.zeroTurnsMotor,
            "--release-after-move",
        ]
        if !moveForm.timeoutSeconds.trimmingCharacters(in: .whitespaces).isEmpty {
            args.append(contentsOf: ["--timeout-s", moveForm.timeoutSeconds])
        }
        await run(
            action: "move-continuous-async",
            arguments: args,
            countsAsBlocking: false,
            storePrimaryResponse: true,
            storeTelemetryResponse: true
        )
    }
}

struct StatusBadge: View {
    let title: String
    let value: String
    let color: Color

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(title)
                .font(.caption)
                .foregroundStyle(.secondary)
            Text(value)
                .font(.headline)
                .padding(.horizontal, 10)
                .padding(.vertical, 6)
                .frame(maxWidth: .infinity, alignment: .leading)
                .background(color.opacity(0.18), in: RoundedRectangle(cornerRadius: 10, style: .continuous))
        }
    }
}

struct LabeledInputField: View {
    let title: String
    @Binding var text: String

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            Text(title)
                .font(.caption.weight(.medium))
                .foregroundStyle(.secondary)
            TextField(title, text: $text)
                .textFieldStyle(.roundedBorder)
        }
    }
}

struct TelemetryGraphView: View {
    let samples: [TelemetrySample]
    let metric: TelemetryMetric

    private var points: [Double] {
        samples.compactMap { metric.value(for: $0) }
    }

    var body: some View {
        GeometryReader { proxy in
            let width = proxy.size.width
            let height = max(1, proxy.size.height)
            let values = points
            let minValue = values.min() ?? 0
            let maxValue = values.max() ?? 1
            let span = max(1e-9, maxValue - minValue)

            ZStack {
                RoundedRectangle(cornerRadius: 12, style: .continuous)
                    .fill(Color(nsColor: .textBackgroundColor))
                if minValue <= 0, maxValue >= 0 {
                    let zeroY = height - CGFloat((0 - minValue) / span) * height
                    Path { path in
                        path.move(to: CGPoint(x: 0, y: zeroY))
                        path.addLine(to: CGPoint(x: width, y: zeroY))
                    }
                    .stroke(Color.secondary.opacity(0.25), style: StrokeStyle(lineWidth: 1, dash: [4, 4]))
                }
                Path { path in
                    for (index, value) in values.enumerated() {
                        let x = values.count > 1 ? (CGFloat(index) / CGFloat(values.count - 1)) * width : width / 2
                        let y = height - CGFloat((value - minValue) / span) * height
                        if index == 0 {
                            path.move(to: CGPoint(x: x, y: y))
                        } else {
                            path.addLine(to: CGPoint(x: x, y: y))
                        }
                    }
                }
                .stroke(Color.accentColor, style: StrokeStyle(lineWidth: 2, lineCap: .round, lineJoin: .round))

                VStack {
                    HStack {
                        Text(String(format: "%.4g %@", maxValue, metric.unit))
                            .font(.system(.caption2, design: .monospaced))
                            .foregroundStyle(.secondary)
                        Spacer()
                    }
                    Spacer()
                    HStack {
                        Text(String(format: "%.4g %@", minValue, metric.unit))
                            .font(.system(.caption2, design: .monospaced))
                            .foregroundStyle(.secondary)
                        Spacer()
                    }
                }
                .padding(8)
            }
        }
    }
}

struct BackendSidebarView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    var body: some View {
        ScrollView {
            Form {
                Section("Backend") {
                    LabeledInputField(title: "Repo root", text: $vm.repoRoot)
                    Stepper("Axis \(vm.axisIndex)", value: $vm.axisIndex, in: 0...1)
                    LabeledInputField(title: "KV estimate", text: $vm.kvEstimate)
                    LabeledInputField(title: "Line-line resistance (ohm)", text: $vm.lineLineROhm)
                    LabeledInputField(title: "Clear-errors settle (s)", text: $vm.settleSeconds)
                    Toggle("Debug backend", isOn: $vm.debugMode)
                }

                Section("Actions") {
                    Button("Refresh Status") { Task { await vm.refreshStatus() } }
                    Button("Diagnose") { Task { await vm.runDiagnose() } }
                    Button("Motor Fact Sheet") { Task { await vm.runFactSheet() } }
                    Button("Startup (state 3)") { Task { await vm.startup() } }
                        .disabled(vm.isBusy || vm.capabilities?.can_startup == false)
                    Button("Idle") { Task { await vm.idle() } }
                        .disabled(vm.isBusy || vm.capabilities?.can_idle == false)
                    Button("Clear Errors") { Task { await vm.clearErrors() } }
                        .disabled(vm.isBusy || vm.capabilities?.can_clear_errors == false)
                }
            }
            .formStyle(.grouped)
        }
        .frame(minWidth: 300, idealWidth: 340, maxWidth: 380)
    }
}

struct TelemetrySectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @ObservedObject var live: LiveMonitorModel

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Text("Telemetry")
                    .font(.title2.bold())
                Spacer()
                Button("Poll Now") {
                    Task { await vm.pollTelemetryOnce() }
                }
                .disabled(vm.blockingActionInFlight)
            }

            Text("Realtime in-app graph from the persistent backend event stream. 'Poll Now' is just a manual snapshot.")
                .foregroundStyle(.secondary)

            HStack {
                Picker("Metric", selection: $vm.telemetryMetric) {
                    ForEach(TelemetryMetric.allCases) { metric in
                        Text(metric.rawValue).tag(metric)
                    }
                }
                .pickerStyle(.segmented)

                Toggle(
                    "Live chart",
                    isOn: Binding(
                        get: { vm.telemetryAutoRefresh },
                        set: { newValue in
                            vm.telemetryAutoRefresh = newValue
                            vm.telemetryAutoRefreshChanged(newValue)
                        }
                    )
                )
            }

            if live.telemetrySamples.isEmpty {
                Text("No telemetry samples yet.")
                    .foregroundStyle(.secondary)
            } else {
                TelemetryGraphView(samples: live.telemetrySamples, metric: vm.telemetryMetric)
                    .frame(minHeight: 220)
            }

            if let latest = live.telemetrySamples.last {
                HStack(spacing: 18) {
                    Text(String(format: "pos %.4f t", latest.posEst))
                    Text(String(format: "err %.4f t", latest.trackingError))
                    Text(String(format: "Iq %.3f A", latest.iqMeas))
                    if let tq = latest.estimatedMotorTorqueNm {
                        Text(String(format: "est tq %.4f Nm", tq))
                    }
                    Text(String(format: "vel %.3f t/s", latest.velEst))
                }
                .font(.system(.caption, design: .monospaced))
                .foregroundStyle(.secondary)
            }
        }
        .padding(16)
        .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
    }
}

struct ReadinessGridView: View {
    @ObservedObject var live: LiveMonitorModel
    let diagnosis: BackendDiagnosis?
    let fallbackSnapshot: BackendSnapshot?
    let fallbackDevice: BackendDevice?

    private var capabilities: BackendCapabilities? { live.capabilities }
    private var snapshot: BackendSnapshot? { live.snapshot ?? fallbackSnapshot }
    private var device: BackendDevice? { live.device ?? fallbackDevice }

    var body: some View {
        let startupColor: Color = (capabilities?.startup_ready == true) ? .green : .orange
        let errorColor: Color = (capabilities?.has_latched_errors == true) ? .red : .green
        let armedColor: Color = (capabilities?.armed == true) ? .blue : .gray
        let idleColor: Color = (capabilities?.idle == true) ? .green : .gray
        let motionColor: Color = (capabilities?.motion_active == true) ? .orange : .gray
        return VStack(alignment: .leading, spacing: 12) {
            Text("State at a Glance")
                .font(.title2.bold())
            LazyVGrid(columns: [GridItem(.adaptive(minimum: 220), spacing: 12)], spacing: 12) {
                StatusBadge(title: "Verdict", value: diagnosis?.verdict ?? "unknown", color: startupColor)
                StatusBadge(title: "Startup", value: (capabilities?.startup_ready == true) ? "ready" : "not ready", color: startupColor)
                StatusBadge(title: "Latched Errors", value: (capabilities?.has_latched_errors == true) ? "present" : "none", color: errorColor)
                StatusBadge(title: "Axis State", value: snapshot?.state.map(String.init) ?? "unknown", color: armedColor)
                StatusBadge(title: "Armed", value: (capabilities?.armed == true) ? "closed-loop" : "not armed", color: armedColor)
                StatusBadge(title: "Idle", value: (capabilities?.idle == true) ? "idle" : "not idle", color: idleColor)
                StatusBadge(title: "Motion", value: (capabilities?.motion_active == true) ? "background move active" : "no background move", color: motionColor)
                StatusBadge(title: "Encoder Ready", value: (snapshot?.enc_ready == true) ? "true" : "false", color: (snapshot?.enc_ready == true) ? .green : .orange)
                StatusBadge(title: "Index Found", value: (snapshot?.enc_index_found == true) ? "true" : "false", color: (snapshot?.enc_index_found == true) ? .green : .orange)
                StatusBadge(title: "Pos Estimate", value: snapshot?.pos_est.map { String(format: "%.6f t", $0) } ?? "unknown", color: .gray)
                StatusBadge(title: "Vbus", value: device?.vbus_voltage.map { String(format: "%.2f V", $0) } ?? "unknown", color: .gray)
            }
        }
    }
}

struct ProfileEditorSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Text("Profile Editor")
                    .font(.title3.bold())
                Spacer()
                Button("Reload Selected") {
                    Task { await vm.loadProfileEditor() }
                }
                .disabled(vm.moveForm.profileName.isEmpty)
                Button("Save Profile") {
                    Task { await vm.saveProfileEditor() }
                }
                .buttonStyle(.borderedProminent)
                .disabled(vm.profileEditor.name.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty)
            }

            Text("The editor is preloaded from the selected dropdown profile. Save with the same name to overwrite it, or change the name to fork a new profile.")
                .font(.caption)
                .foregroundStyle(.secondary)

            VStack(alignment: .leading, spacing: 8) {
                Text("Identity")
                    .font(.headline)
                HStack(alignment: .top, spacing: 12) {
                    LabeledInputField(title: "Profile name", text: $vm.profileEditor.name)
                    LabeledInputField(title: "Source", text: $vm.profileEditor.source)
                    LabeledInputField(title: "Load mode", text: $vm.profileEditor.loadMode)
                }
                Text("Loaded from: \(vm.profileEditor.loadedProfileName.isEmpty ? "none" : vm.profileEditor.loadedProfileName)")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                Text("Notes")
                    .font(.subheadline.weight(.medium))
                TextEditor(text: $vm.profileEditor.notes)
                    .font(.system(.body, design: .default))
                    .frame(minHeight: 70)
                    .padding(6)
                    .background(Color(nsColor: .textBackgroundColor), in: RoundedRectangle(cornerRadius: 8, style: .continuous))
                Text("Limitations (one per line)")
                    .font(.subheadline.weight(.medium))
                TextEditor(text: $vm.profileEditor.limitationsText)
                    .font(.system(.body, design: .default))
                    .frame(minHeight: 80)
                    .padding(6)
                    .background(Color(nsColor: .textBackgroundColor), in: RoundedRectangle(cornerRadius: 8, style: .continuous))
            }

            VStack(alignment: .leading, spacing: 8) {
                Text("Behavior Flags")
                    .font(.headline)
                Toggle("Require repeatability", isOn: $vm.profileEditor.requireRepeatability)
                Toggle("Stop on frame jump", isOn: $vm.profileEditor.stopOnFrameJump)
                Toggle("Stop on hard fault", isOn: $vm.profileEditor.stopOnHardFault)
                Toggle("Enable overspeed error", isOn: $vm.profileEditor.enableOverspeedError)
                Toggle("Quiet hold enable", isOn: $vm.profileEditor.quietHoldEnable)
                Toggle("Quiet hold persist", isOn: $vm.profileEditor.quietHoldPersist)
                Toggle("Fail to idle in move helper", isOn: $vm.profileEditor.failToIdle)
                Toggle("Disable quiet hold reanchor", isOn: $vm.profileEditor.quietHoldReanchorDisabled)
            }

            VStack(alignment: .leading, spacing: 8) {
                Text("Motion Gains / Limits")
                    .font(.headline)
                HStack(alignment: .top, spacing: 12) {
                    LabeledInputField(title: "Current lim", text: $vm.profileEditor.currentLim)
                    LabeledInputField(title: "Pos gain", text: $vm.profileEditor.posGain)
                    LabeledInputField(title: "Vel gain", text: $vm.profileEditor.velGain)
                    LabeledInputField(title: "Vel I gain", text: $vm.profileEditor.velIGain)
                }
                HStack(alignment: .top, spacing: 12) {
                    LabeledInputField(title: "Trap vel", text: $vm.profileEditor.trapVel)
                    LabeledInputField(title: "Trap acc", text: $vm.profileEditor.trapAcc)
                    LabeledInputField(title: "Trap dec", text: $vm.profileEditor.trapDec)
                }
                HStack(alignment: .top, spacing: 12) {
                    LabeledInputField(title: "Vel limit", text: $vm.profileEditor.velLimit)
                    LabeledInputField(title: "Vel limit tol", text: $vm.profileEditor.velLimitTolerance)
                    LabeledInputField(title: "Stiction kick Nm", text: $vm.profileEditor.stictionKickNm)
                }
            }

            VStack(alignment: .leading, spacing: 8) {
                Text("Target / Settle")
                    .font(.headline)
                HStack(alignment: .top, spacing: 12) {
                    LabeledInputField(title: "Target tol turns", text: $vm.profileEditor.targetToleranceTurns)
                    LabeledInputField(title: "Target vel tol", text: $vm.profileEditor.targetVelToleranceTurnsS)
                    LabeledInputField(title: "Timeout s", text: $vm.profileEditor.timeoutS)
                }
                HStack(alignment: .top, spacing: 12) {
                    LabeledInputField(title: "Min delta turns", text: $vm.profileEditor.minDeltaTurns)
                    LabeledInputField(title: "Settle s", text: $vm.profileEditor.settleS)
                }
            }

            VStack(alignment: .leading, spacing: 8) {
                Text("Quiet Hold")
                    .font(.headline)
                HStack(alignment: .top, spacing: 12) {
                    LabeledInputField(title: "Quiet hold s", text: $vm.profileEditor.quietHoldS)
                    LabeledInputField(title: "Pos gain scale", text: $vm.profileEditor.quietHoldPosGainScale)
                    LabeledInputField(title: "Vel gain scale", text: $vm.profileEditor.quietHoldVelGainScale)
                }
                HStack(alignment: .top, spacing: 12) {
                    LabeledInputField(title: "Vel I gain", text: $vm.profileEditor.quietHoldVelIGain)
                    LabeledInputField(title: "Vel limit scale", text: $vm.profileEditor.quietHoldVelLimitScale)
                    LabeledInputField(title: "Reanchor err turns", text: $vm.profileEditor.quietHoldReanchorErrTurns)
                        .disabled(vm.profileEditor.quietHoldReanchorDisabled)
                }
            }
        }
        .padding(14)
        .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
    }
}

struct MoveSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @ObservedObject var live: LiveMonitorModel

    private var capabilities: BackendCapabilities? { live.capabilities ?? vm.response?.capabilities }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Continuous Move")
                .font(.title2.bold())
            Text("This now launches as a background move so telemetry can keep updating during travel. No segmented waypoint mode.")
                .foregroundStyle(.secondary)

            HStack(alignment: .top, spacing: 12) {
                LabeledInputField(title: "Angle (deg)", text: $vm.moveForm.angleDeg)
                LabeledInputField(title: "Gear ratio", text: $vm.moveForm.gearRatio)
                LabeledInputField(title: "Timeout (s, optional)", text: $vm.moveForm.timeoutSeconds)
            }
            Picker("Angle space", selection: $vm.moveForm.angleSpace) {
                Text("Gearbox output").tag("gearbox_output")
                Text("Motor").tag("motor")
            }
            .pickerStyle(.segmented)

            Toggle("Relative to current", isOn: $vm.moveForm.relativeToCurrent)
            Toggle(
                "Release to IDLE after move",
                isOn: Binding(
                    get: { vm.moveForm.releaseAfterMove },
                    set: { newValue in
                        vm.moveForm.releaseAfterMove = newValue
                        vm.releaseAfterMoveChanged(newValue)
                    }
                )
            )
            HStack(alignment: .top, spacing: 12) {
                LabeledInputField(title: "Zero turns motor (absolute mode)", text: $vm.moveForm.zeroTurnsMotor)
                    .disabled(vm.moveForm.relativeToCurrent)
                Button("Capture Current as Zero") {
                    vm.captureZeroHere()
                }
                .disabled(vm.isBusy || capabilities?.can_capture_zero_here == false)
            }

            Picker("Profile", selection: $vm.moveForm.profileName) {
                ForEach(vm.profiles, id: \.self) { profile in
                    Text(profile).tag(profile)
                }
            }
            .disabled(vm.profiles.isEmpty)

            if let detail = vm.profileDetails.first(where: { $0.name == vm.moveForm.profileName }) {
                VStack(alignment: .leading, spacing: 8) {
                    Text("Selected Profile")
                        .font(.headline)
                    if let notes = detail.notes, !notes.isEmpty {
                        Text(notes)
                    }
                    if let limitations = detail.limitations, !limitations.isEmpty {
                        ForEach(limitations, id: \.self) { item in
                            Text("- \(item)")
                                .foregroundStyle(.secondary)
                        }
                    }
                }
                .padding(12)
                .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
            }

            ProfileEditorSectionView(vm: vm)

            Button("Run Continuous Move") {
                Task { await vm.moveContinuous() }
            }
            .buttonStyle(.borderedProminent)
            .disabled(vm.isBusy || capabilities?.can_move_continuous != true || capabilities?.motion_active == true)
        }
        .padding(16)
        .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
    }
}

struct SliderFollowSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @ObservedObject var live: LiveMonitorModel

    private var capabilities: BackendCapabilities? { live.capabilities ?? vm.response?.capabilities }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Live Angle Slider")
                .font(.title2.bold())
            Text("Absolute gearbox-output target from 0° to 360°. This issues debounced non-blocking target updates. Capture a zero anchor first.")
                .foregroundStyle(.secondary)

            HStack {
                Text(String(format: "%.0f°", vm.sliderFollow.angleDeg))
                    .font(.system(.title3, design: .monospaced))
                Spacer()
                if !vm.hasAbsoluteZeroAnchor {
                    Text("Zero anchor required")
                        .foregroundStyle(.orange)
                }
            }

            Slider(
                value: Binding(
                    get: { vm.sliderFollow.angleDeg },
                    set: { newValue in
                        vm.sliderFollow.angleDeg = newValue
                        vm.sliderAngleDidChange()
                    }
                ),
                in: 0...360,
                step: 1
            )
            .disabled(capabilities?.can_move_continuous != true || !vm.hasAbsoluteZeroAnchor)

            HStack {
                Text("0°")
                    .foregroundStyle(.secondary)
                Spacer()
                Text("360°")
                    .foregroundStyle(.secondary)
            }

            VStack(alignment: .leading, spacing: 8) {
                Text("Presets")
                    .font(.headline)
                LazyVGrid(columns: [GridItem(.adaptive(minimum: 58), spacing: 8)], spacing: 8) {
                    ForEach(OperatorConsoleViewModel.sliderPresetAngles, id: \.self) { angle in
                        Button(String(format: "%.0f°", angle)) {
                            Task { await vm.selectSliderPreset(angle) }
                        }
                        .buttonStyle(.bordered)
                        .disabled(vm.isBusy || capabilities?.can_move_continuous != true || !vm.hasAbsoluteZeroAnchor)
                    }
                }
            }

            Toggle(
                "Live follow while dragging",
                isOn: Binding(
                    get: { vm.sliderFollow.liveEnabled },
                    set: { newValue in
                        vm.sliderFollow.liveEnabled = newValue
                        Task { await vm.sliderFollowToggled(newValue) }
                    }
                )
            )
            .disabled(capabilities?.can_move_continuous != true || !vm.hasAbsoluteZeroAnchor || vm.moveForm.releaseAfterMove)

            HStack {
                Button("Send Slider Target Now") {
                    Task { await vm.sendSliderTargetNow() }
                }
                .disabled(vm.isBusy || capabilities?.can_move_continuous != true || !vm.hasAbsoluteZeroAnchor)

                Button("Capture Current as 0°") {
                    vm.captureZeroHere()
                }
                .disabled(vm.isBusy || capabilities?.can_capture_zero_here == false)
            }

            Text(
                vm.moveForm.releaseAfterMove
                ? "With release mode on, presets and 'Send Slider Target Now' run as one-shot moves and then IDLE. Live follow is disabled because follow-plus-idle on every update is not a coherent control mode."
                : "The slider and presets use the selected profile's travel gains and limits. They do not run the one-shot settle/verification path, and they do not auto-idle afterward."
            )
            .font(.caption)
            .foregroundStyle(.secondary)
        }
        .padding(16)
        .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
    }
}

struct FactSectionView: View {
    let title: String
    let rows: [FactRow]

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text(title)
                .font(.headline)
            ForEach(rows) { row in
                VStack(alignment: .leading, spacing: 2) {
                    HStack(alignment: .firstTextBaseline) {
                        Text(row.label)
                            .font(.system(.body, design: .monospaced))
                        Spacer()
                        Text(row.value?.description ?? "unknown")
                            .font(.system(.body, design: .monospaced))
                    }
                    if !row.note.isEmpty {
                        Text(row.note)
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }
                .padding(.vertical, 4)
                Divider()
            }
        }
        .padding(12)
        .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 14, style: .continuous))
    }
}

struct ContentView: View {
    @StateObject private var vm = OperatorConsoleViewModel()

    var body: some View {
        HSplitView {
            BackendSidebarView(vm: vm)

            ScrollView {
                VStack(alignment: .leading, spacing: 18) {
                    header
                    readinessGrid
                    telemetrySection
                    moveSection
                    sliderFollowSection
                    diagnosisSection
                    factSheetSection
                    rawResultSection
                }
                .padding(20)
            }
        }
        .frame(minWidth: 1180, minHeight: 860)
        .task {
            await vm.ensureInitialRefresh()
        }
        .task(id: vm.moveForm.profileName) {
            guard !vm.moveForm.profileName.isEmpty else { return }
            await vm.loadProfileEditor(name: vm.moveForm.profileName)
        }
    }

    private var header: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("Motor / Board Operator Console")
                    .font(.largeTitle.bold())
                Spacer()
                if vm.isBusy {
                    ProgressView()
                }
            }
            Text("SwiftUI front-end. Python backend remains the authoritative control path.")
                .foregroundStyle(.secondary)

            if let error = vm.lastClientError {
                Text(error)
                    .foregroundStyle(.red)
            } else if let message = vm.response?.message {
                Text(message)
                    .foregroundColor(vm.response?.ok == true ? .secondary : .red)
            }
        }
    }

    private var telemetrySection: some View {
        TelemetrySectionView(vm: vm, live: vm.liveMonitor)
    }

    private var readinessGrid: some View {
        ReadinessGridView(
            live: vm.liveMonitor,
            diagnosis: vm.diagnosis,
            fallbackSnapshot: vm.response?.snapshot,
            fallbackDevice: vm.response?.device
        )
    }

    private var moveSection: some View {
        MoveSectionView(vm: vm, live: vm.liveMonitor)
    }

    private var sliderFollowSection: some View {
        SliderFollowSectionView(vm: vm, live: vm.liveMonitor)
    }

    private var diagnosisSection: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Diagnosis")
                .font(.title2.bold())
            if let diagnosis = vm.diagnosis?.diagnosis {
                Text(diagnosis)
            }
            if let commands = vm.diagnosis?.commands, !commands.isEmpty {
                Text("Next Commands")
                    .font(.headline)
                ForEach(commands, id: \.self) { cmd in
                    Text(cmd)
                        .font(.system(.body, design: .monospaced))
                }
            }
            if let notes = vm.diagnosis?.notes, !notes.isEmpty {
                Text("Notes")
                    .font(.headline)
                ForEach(notes, id: \.self) { note in
                    Text("- \(note)")
                }
            }
        }
        .padding(16)
        .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
    }

    private var factSheetSection: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Motor Fact Sheet")
                .font(.title2.bold())
            if let measured = vm.response?.fact_sheet?.measured_live, !measured.isEmpty {
                FactSectionView(title: "Measured Live", rows: measured)
            }
            if let configured = vm.response?.fact_sheet?.configured, !configured.isEmpty {
                FactSectionView(title: "Configured", rows: configured)
            }
            if let quickChecks = vm.response?.fact_sheet?.quick_checks, !quickChecks.isEmpty {
                FactSectionView(title: "Quick Checks", rows: quickChecks)
            }
        }
    }

    private var rawResultSection: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Backend Result")
                .font(.title2.bold())
            Text(vm.response?.resultSummary ?? vm.lastClientError ?? "No response yet")
                .font(.system(.body, design: .monospaced))
                .textSelection(.enabled)
            if let rawJSON = vm.response?.rawJSON, !rawJSON.isEmpty {
                ScrollView {
                    Text(rawJSON)
                        .font(.system(.caption, design: .monospaced))
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .textSelection(.enabled)
                        .padding(8)
                }
                .frame(minHeight: 220)
                .background(Color(nsColor: .textBackgroundColor))
                .overlay(
                    RoundedRectangle(cornerRadius: 6, style: .continuous)
                        .stroke(Color.secondary.opacity(0.25))
                )
            }
        }
        .padding(16)
        .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
    }
}
