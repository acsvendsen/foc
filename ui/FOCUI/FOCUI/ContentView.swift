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

enum TelemetryStreamRate: String, CaseIterable, Identifiable {
    case slow = "Slow"
    case medium = "Medium"
    case fast = "Fast"

    var id: String { rawValue }

    var intervalMs: Int {
        switch self {
        case .slow:
            return 180
        case .medium:
            return 90
        case .fast:
            return 40
        }
    }
}

enum ConsolePanel: String, CaseIterable, Identifiable {
    case stateAtGlance = "state_at_glance"
    case outputSensor = "output_sensor"
    case guidedBringup = "guided_bringup"
    case telemetry = "telemetry"
    case move = "move"
    case sliderFollow = "slider_follow"
    case dualAxisSync = "dual_axis_sync"
    case diagnosis = "diagnosis"
    case factSheet = "fact_sheet"
    case backendResult = "backend_result"

    var id: String { rawValue }

    var title: String {
        switch self {
        case .stateAtGlance:
            return "State at a Glance"
        case .outputSensor:
            return "Output Sensor"
        case .guidedBringup:
            return "Guided Bring-Up"
        case .telemetry:
            return "Telemetry"
        case .move:
            return "Continuous / Direct Move"
        case .sliderFollow:
            return "Live Angle Slider"
        case .dualAxisSync:
            return "Dual-Joint Sync Move"
        case .diagnosis:
            return "Diagnosis"
        case .factSheet:
            return "Motor Fact Sheet"
        case .backendResult:
            return "Backend Result"
        }
    }

    var subtitle: String {
        switch self {
        case .stateAtGlance:
            return "Readiness and recovery"
        case .outputSensor:
            return "External output-angle bridge"
        case .guidedBringup:
            return "Bounded staged bring-up"
        case .telemetry:
            return "Motor-side graph and snapshots"
        case .move:
            return "Profiles, direct control, run quality"
        case .sliderFollow:
            return "Absolute output target slider"
        case .dualAxisSync:
            return "Two-axis move testing"
        case .diagnosis:
            return "Backend diagnosis text"
        case .factSheet:
            return "Measured/configured motor facts"
        case .backendResult:
            return "Raw backend response"
        }
    }

    static var defaultVisibleRaw: String {
        allCases.map(\.rawValue).joined(separator: ",")
    }
}

struct DirectRunBaseline {
    let mode: String
    let startedAt: Date
    let startMotorTurns: Double?
    let startOutputTurns: Double?
    let startOutputVelTurnsS: Double?
    let startLagOutputTurns: Double?
    let requestedMotorTurns: Double?
    let requestedTurnsPerSecond: Double?
    let requestedDurationS: Double?
    let relative: Bool
    let releaseAfter: Bool
}

struct DirectRunQuality {
    enum Verdict: String {
        case good
        case partial
        case stalled
        case sensorMismatch
        case signInverted
        case wrongDirection
        case faulted
        case informational
    }

    let mode: String
    let verdict: Verdict
    let headline: String
    let explanation: String
    let expectedMotorDeltaTurns: Double?
    let actualMotorDeltaTurns: Double?
    let furthestMotorDeltaTurns: Double?
    let maxAbsMotorDeltaTurns: Double?
    let motorFollowFraction: Double?
    let expectedOutputDeltaTurns: Double?
    let expectedOutputFromActualMotorTurns: Double?
    let actualOutputDeltaTurns: Double?
    let furthestOutputDeltaTurns: Double?
    let maxAbsOutputDeltaTurns: Double?
    let outputFollowFraction: Double?
    let transmissionFollowFraction: Double?
    let directionCorrect: Bool?
    let motorReachedTarget: Bool?
    let finalOutputVelTurnsS: Double?
    let lagDeltaOutputTurns: Double?
    let peakMotorVelTurnsS: Double?
    let peakOutputVelTurnsS: Double?
    let errorSummary: String?
    let assistUsed: Bool
    let assistFloorTurnsPerSecond: Double?
    let assistKickTurnsPerSecond: Double?
    let assistKickDurationS: Double?
    let assistBreakawayDetected: Bool?
    let assistBreakawayDetectedAtS: Double?
}

struct VelocitySweepPoint {
    enum Tier: String {
        case belowFloor
        case borderline
        case usable
        case faulted
    }

    let commandTurnsPerSecond: Double
    let quality: DirectRunQuality
    let tier: Tier
}

struct VelocitySweepSummary {
    let points: [VelocitySweepPoint]
    let breakawayFloorTurnsPerSecond: Double?
    let usableFloorTurnsPerSecond: Double?
    let stoppedReason: String?
    let completedAt: Date
}

struct DirectVelocityHint {
    enum Tier {
        case unknown
        case belowFloor
        case borderline
        case usable
    }

    let tier: Tier
    let headline: String
    let detail: String
}

@MainActor
final class LiveMonitorModel: ObservableObject {
    private static let maxTelemetrySamples = 60

    @Published var snapshot: BackendSnapshot?
    @Published var capabilities: BackendCapabilities?
    @Published var device: BackendDevice?
    @Published var outputSensor: BackendOutputSensor?
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
        if let outputSensor = result.output_sensor {
            self.outputSensor = outputSensor
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

    func reset() {
        snapshot = nil
        capabilities = nil
        device = nil
        outputSensor = nil
        clearTelemetryHistory()
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
    private static let legacyDefaultProfileName = "gearbox_output_continuous_quiet_20260309"
    private static let preferredMksStartupProfileName = "mks_mounted_direct_preload_v3"

    static let sliderPresetAngles: [Double] = [0, 30, 45, 60, 90, 120, 180, 270, 360]

    @Published var repoRoot: String
    @Published var axisIndex: Int = 0
    @Published var selectedBoardSerial: String = ""
    @Published var detectedBoardSerials: [String] = []
    @Published var kvEstimate: String = "140"
    @Published var lineLineROhm: String = "0.30"
    @Published var settleSeconds: String = "0.15"
    @Published var debugMode: Bool = false
    @Published var motorDirectionSelection: Int = 1

    @Published var moveForm = MoveFormState()
    @Published var directControlForm = DirectControlFormState()
    @Published var syncMoveForm = SyncMoveFormState()
    @Published var profileEditor = ProfileEditorFormState()
    @Published var sliderFollow = SliderFollowState()
    @Published var response: BackendResponse?
    @Published var autoDirectionResponse: BackendResponse?
    @Published var syncAxisAResponse: BackendResponse?
    @Published var syncAxisBResponse: BackendResponse?
    @Published var syncResultResponse: BackendResponse?
    @Published var guidedBringupResponse: BackendResponse?
    @Published var telemetryAutoRefresh = false
    @Published var telemetryMetric: TelemetryMetric = .trackingError
    @Published var telemetryStreamRate: TelemetryStreamRate = .slow
    @Published var blockingActionInFlight = false
    @Published var lastClientError: String?
    @Published var guidedBringupPersistDirection = false
    @Published var activeDirectRunBaseline: DirectRunBaseline?
    @Published var latestDirectRunQuality: DirectRunQuality?
    @Published var latestVelocitySweep: VelocitySweepSummary?

    private let backend = BackendClient()
    let liveMonitor = LiveMonitorModel()
    private var sliderDebounceTask: Task<Void, Never>?
    private var sliderQueuedAngle: Double?
    private var sliderCommandActive = false
    private var telemetryRequestActive = false
    private var backendEventTask: Task<Void, Never>?
    private var initialRefreshDone = false
    private var streamingStarted = false
    private var motorDirectionDirty = false

    init() {
        self.repoRoot = backend.detectRepoRoot()
        self.syncMoveForm.profileName = moveForm.profileName
        self.syncMoveForm.profileAName = moveForm.profileName
        self.syncMoveForm.profileBName = moveForm.profileName
    }

    private func requestContext(axisIndex overrideAxisIndex: Int? = nil, deviceSerial overrideDeviceSerial: String? = nil) -> BackendClient.RequestContext {
        BackendClient.RequestContext(
            repoRoot: repoRoot,
            axisIndex: overrideAxisIndex ?? axisIndex,
            deviceSerial: overrideDeviceSerial ?? selectedBoardSerial,
            kvEstimate: kvEstimate,
            lineLineROhm: lineLineROhm,
            settleSeconds: settleSeconds,
            debug: debugMode
        )
    }

    var isBusy: Bool { blockingActionInFlight }
    var capabilities: BackendCapabilities? { liveMonitor.capabilities ?? response?.capabilities }
    var outputSensor: BackendOutputSensor? { liveMonitor.outputSensor ?? response?.output_sensor }
    private var resolvedGearRatio: Double {
        let raw = moveForm.gearRatio.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let value = Double(raw), value > 0 else { return 25.0 }
        return value
    }
    var outputSensorPortEnv: String? {
        let raw = ProcessInfo.processInfo.environment["ROBOT_OUTPUT_SENSOR_PORT"]?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        return raw.isEmpty ? nil : raw
    }
    private var manualAssistFloorTurnsPerSecond: Double? {
        let raw = directControlForm.assistManualFloorTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let value = Double(raw), value > 0 else { return nil }
        return value
    }
    var detectedBreakawayFloorTurnsPerSecond: Double? { latestVelocitySweep?.breakawayFloorTurnsPerSecond }
    var detectedUsableFloorTurnsPerSecond: Double? { latestVelocitySweep?.usableFloorTurnsPerSecond }
    var assistFloorTurnsPerSecond: Double? {
        manualAssistFloorTurnsPerSecond ?? detectedUsableFloorTurnsPerSecond ?? detectedBreakawayFloorTurnsPerSecond
    }
    var assistFloorSourceLabel: String? {
        if manualAssistFloorTurnsPerSecond != nil {
            return "manual override"
        }
        if detectedUsableFloorTurnsPerSecond != nil {
            return "usable floor"
        }
        if detectedBreakawayFloorTurnsPerSecond != nil {
            return "breakaway floor"
        }
        return nil
    }
    var currentVelocityHint: DirectVelocityHint {
        let raw = directControlForm.turnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let requestedTurnsPerSecond = Double(raw), abs(requestedTurnsPerSecond) > 1e-6 else {
            return DirectVelocityHint(
                tier: .unknown,
                headline: "Need a velocity value",
                detail: "Enter a nonzero timed motor velocity before judging low-speed quality."
            )
        }
        let magnitude = abs(requestedTurnsPerSecond)
        if let usable = detectedUsableFloorTurnsPerSecond {
            if magnitude >= usable {
                return DirectVelocityHint(
                    tier: .usable,
                    headline: "Usable",
                    detail: String(format: "Command is at or above the tested usable floor of %.2f t/s.", usable)
                )
            }
        }
        if let breakaway = detectedBreakawayFloorTurnsPerSecond {
            if magnitude < breakaway {
                return DirectVelocityHint(
                    tier: .belowFloor,
                    headline: "Below floor",
                    detail: String(format: "Command is below the detected breakaway floor of %.2f t/s.", breakaway)
                )
            }
            return DirectVelocityHint(
                tier: .borderline,
                headline: "Borderline",
                detail: detectedUsableFloorTurnsPerSecond.map {
                    String(format: "Command is above breakaway (%.2f t/s) but still below the tested usable floor of %.2f t/s.", breakaway, $0)
                } ?? String(format: "Command is above breakaway (%.2f t/s), but no usable floor has been proven yet.", breakaway)
            )
        }
        return DirectVelocityHint(
            tier: .unknown,
            headline: "No floor yet",
            detail: "Run the automatic breakaway sweep first. Until then, low-speed commands are guesswork."
        )
    }
    var diagnosis: BackendDiagnosis? { response?.diagnosis }
    var snapshot: BackendSnapshot? { liveMonitor.snapshot ?? response?.snapshot }
    var profiles: [String] { response?.available_profiles ?? [] }
    var profileDetails: [BackendProfileDetail] { response?.available_profile_details ?? [] }
    private var readinessCapabilities: BackendCapabilities? { liveMonitor.capabilities ?? response?.capabilities }
    private var readinessSnapshot: BackendSnapshot? { liveMonitor.snapshot ?? response?.snapshot }
    var selectedProfileDetail: BackendProfileDetail? { profileDetails.first(where: { $0.name == moveForm.profileName }) }
    var selectedProfileEditorLoaded: Bool { profileEditor.loadedProfileName == moveForm.profileName }
    var selectedProfileMoveMode: String {
        selectedProfileEditorLoaded
            ? profileEditor.moveMode.trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
            : ""
    }
    var selectedProfileSupportsRuntimeSpeedTweak: Bool {
        selectedProfileMoveMode == "mks_directional_slew_direct"
            || selectedProfileMoveMode == "mks_directional_velocity_travel_direct"
            || selectedProfileMoveMode == "mks_velocity_point_to_point_direct"
    }
    var hasAbsoluteZeroAnchor: Bool { !moveForm.zeroTurnsMotor.trimmingCharacters(in: .whitespaces).isEmpty }
    var currentMotorDirection: Int? { snapshot?.motor_direction }
    var syncAxisASnapshot: BackendSnapshot? { syncAxisAResponse?.snapshot }
    var syncAxisBSnapshot: BackendSnapshot? { syncAxisBResponse?.snapshot }
    var syncAxisACapabilities: BackendCapabilities? { syncAxisAResponse?.capabilities }
    var syncAxisBCapabilities: BackendCapabilities? { syncAxisBResponse?.capabilities }
    private func normalizedSyncSerial(_ serial: String) -> String? {
        let trimmed = serial.trimmingCharacters(in: .whitespacesAndNewlines)
        return trimmed.isEmpty ? nil : trimmed
    }

    var syncAxisTargetsConflict: Bool {
        guard syncMoveForm.axisAIndex == syncMoveForm.axisBIndex else { return false }
        let serialA = normalizedSyncSerial(syncMoveForm.serialA)
        let serialB = normalizedSyncSerial(syncMoveForm.serialB)
        if let serialA, let serialB {
            return serialA == serialB
        }
        return true
    }

    var syncMoveDisabledReason: String? {
        if isBusy {
            return "Another action is running."
        }
        if syncAxisTargetsConflict {
            return "Axis A and Axis B currently resolve to the same board/axis. Same axis index is only allowed when board serials differ."
        }
        if syncAxisACapabilities?.startup_ready != true {
            return "Axis A is not startup-ready."
        }
        if syncAxisBCapabilities?.startup_ready != true {
            return "Axis B is not startup-ready."
        }
        return nil
    }

    var velocitySweepDisabledReason: String? {
        if isBusy {
            return "Another action is currently running."
        }
        if capabilities?.motion_active == true {
            return "A background move is still active."
        }
        if capabilities?.startup_ready != true {
            return "Axis is not startup-ready."
        }
        if outputSensor?.healthy != true {
            return "Healthy output-sensor telemetry is required for breakaway sweep."
        }
        if Double(directControlForm.sweepDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines)) == nil {
            return "Sweep duration must be numeric."
        }
        if velocitySweepCommandValues() == nil {
            return "Sweep start, stop, and step must define a valid range."
        }
        return nil
    }

    var assistedVelocityDisabledReason: String? {
        if isBusy {
            return "Another action is currently running."
        }
        if capabilities?.motion_active == true {
            return "A background move is still active."
        }
        if capabilities?.startup_ready != true {
            return "Axis is not startup-ready."
        }
        if outputSensor?.healthy != true {
            return "Healthy output-sensor telemetry is required for assisted low-speed mode."
        }
        if directControlForm.durationSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            return "Duration is required for assisted velocity."
        }
        if Double(directControlForm.durationSeconds.trimmingCharacters(in: .whitespacesAndNewlines)) == nil {
            return "Duration must be numeric."
        }
        if assistFloorTurnsPerSecond == nil {
            return "Run sweep first or set a manual floor override."
        }
        return nil
    }

    var stateRepairButtonTitle: String {
        let capabilities = readinessCapabilities
        let snapshot = readinessSnapshot
        if capabilities == nil || snapshot == nil {
            return "Refresh"
        }
        if capabilities?.motion_active == true {
            return "Motion Busy"
        }
        if capabilities?.has_latched_errors == true || capabilities?.startup_ready != true || snapshot?.enc_ready != true {
            return "Make Ready"
        }
        if capabilities?.idle != true {
            return "Set Idle"
        }
        return "Refresh"
    }

    var stateRepairHint: String {
        let capabilities = readinessCapabilities
        let snapshot = readinessSnapshot
        if capabilities == nil || snapshot == nil {
            return "Fetch a fresh motor-state snapshot first."
        }
        if capabilities?.motion_active == true {
            return "A background move is active. Make Ready is blocked until that move finishes."
        }
        if capabilities?.has_latched_errors == true {
            return "Latched errors block direct control and need clearing first."
        }
        if capabilities?.startup_ready != true || snapshot?.enc_ready != true {
            return "The axis is not startup-ready. This runs one bounded backend recovery loop with profile-aware startup repair and an MKS fallback after encoder mismatch."
        }
        if capabilities?.idle != true {
            return "The axis is armed. This returns it to the safe idle baseline."
        }
        return "State already looks usable. This just refreshes the snapshot."
    }

    var readinessErrorSummary: String? {
        var parts: [String] = []
        if let axis = diagnosis?.report?.axis_err_names, !axis.isEmpty {
            parts.append("axis: \(axis.joined(separator: ", "))")
        }
        if let motor = diagnosis?.report?.motor_err_names, !motor.isEmpty {
            parts.append("motor: \(motor.joined(separator: ", "))")
        }
        if let enc = diagnosis?.report?.enc_err_names, !enc.isEmpty {
            parts.append("encoder: \(enc.joined(separator: ", "))")
        }
        if let ctrl = diagnosis?.report?.ctrl_err_names, !ctrl.isEmpty {
            parts.append("controller: \(ctrl.joined(separator: ", "))")
        }
        if !parts.isEmpty {
            return parts.joined(separator: " | ")
        }
        if let snapshot, (snapshot.axis_err ?? 0) != 0 || (snapshot.motor_err ?? 0) != 0 || (snapshot.enc_err ?? 0) != 0 || (snapshot.ctrl_err ?? 0) != 0 {
            return "axis=\(snapshot.axis_err ?? 0), motor=\(snapshot.motor_err ?? 0), encoder=\(snapshot.enc_err ?? 0), controller=\(snapshot.ctrl_err ?? 0)"
        }
        return nil
    }

    private var selectedProfileName: String {
        moveForm.profileName.trimmingCharacters(in: .whitespacesAndNewlines)
    }

    private var preferredMksStartupProfileName: String {
        if selectedProfileName.hasPrefix("mks_") {
            return selectedProfileName
        }
        if profiles.contains(Self.preferredMksStartupProfileName) {
            return Self.preferredMksStartupProfileName
        }
        if let mounted = profiles.first(where: { $0.hasPrefix("mks_mounted_") }) {
            return mounted
        }
        if let anyMks = profiles.first(where: { $0.hasPrefix("mks_") }) {
            return anyMks
        }
        return selectedProfileName
    }

    private var shouldUseMksStartupRepairFallback: Bool {
        if selectedProfileName.hasPrefix("mks_") {
            return true
        }
        let summary = readinessErrorSummary ?? ""
        return summary.contains("ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH")
            || summary.contains("AXIS_ERROR_ENCODER_FAILED")
    }

    func repairAxisState() async {
        blockingActionInFlight = true
        lastClientError = nil
        defer { blockingActionInFlight = false }
        do {
            let fallbackProfile = shouldUseMksStartupRepairFallback ? preferredMksStartupProfileName : ""
            var arguments = ["--timeout-s", "30", "--repair-passes", "5", "--profile-name", selectedProfileName]
            if !fallbackProfile.isEmpty {
                arguments += ["--mks-fallback-profile-name", fallbackProfile]
            }
            let result = try await backend.run(action: "make-ready", arguments: arguments, context: requestContext())
            response = result
            mergeProfilesIfNeeded(from: result)
            mergeProfileEditorIfNeeded(from: result)
            mergeLiveStatusIfNeeded(from: result)
            liveMonitor.appendTelemetrySample(from: result, fallbackTc: response?.snapshot?.tc)

            if let repairProfile = result.result?.objectValue?["repair_profile_name"]?.stringValue,
               !repairProfile.isEmpty,
               repairProfile.hasPrefix("mks_"),
               moveForm.profileName != repairProfile {
                moveForm.profileName = repairProfile
            }

            if result.result?.objectValue?["final_ready"]?.boolValue == true {
                lastClientError = nil
            } else {
                lastClientError = result.message ?? "Make Ready did not reach a healthy idle state."
            }
        } catch {
            lastClientError = error.localizedDescription
        }
    }

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
    func setMotorDirectionSelection(_ direction: Int) {
        motorDirectionSelection = (direction < 0 ? -1 : 1)
        motorDirectionDirty = true
    }

    func autoDetectMotorDirection() async {
        blockingActionInFlight = true
        lastClientError = nil
        defer { blockingActionInFlight = false }
        do {
            let result = try await backend.run(action: "auto-direction-contract", arguments: [], context: requestContext())
            response = result
            autoDirectionResponse = result
            mergeProfilesIfNeeded(from: result)
            mergeProfileEditorIfNeeded(from: result)
            mergeLiveStatusIfNeeded(from: result)
        } catch {
            lastClientError = error.localizedDescription
        }
        motorDirectionDirty = false
    }

    func runGuidedBringup() async {
        blockingActionInFlight = true
        lastClientError = nil
        defer { blockingActionInFlight = false }
        do {
            var args: [String] = []
            if guidedBringupPersistDirection {
                args.append("--persist")
            }
            let result = try await backend.run(action: "guided-bringup", arguments: args, context: requestContext())
            response = result
            guidedBringupResponse = result
            mergeProfilesIfNeeded(from: result)
            mergeProfileEditorIfNeeded(from: result)
            mergeLiveStatusIfNeeded(from: result)
        } catch {
            lastClientError = error.localizedDescription
        }
    }

    func applyMotorDirection() async {
        let direction = (motorDirectionSelection < 0 ? -1 : 1)
        await run(action: "set-motor-direction", arguments: ["--direction", String(direction)])
        motorDirectionDirty = false
    }

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
                await disableStreamingIfAllowed()
            }
        }
    }

    func telemetryStreamRateChanged(_ newRate: TelemetryStreamRate) {
        telemetryStreamRate = newRate
        Task {
            guard telemetryAutoRefresh else { return }
            await disableStreamingIfAllowed(force: true)
            await ensureStreaming()
        }
    }

    private func responseErrorSummary(_ response: BackendResponse) -> String? {
        var parts: [String] = []
        if let axis = response.diagnosis?.report?.axis_err_names, !axis.isEmpty {
            parts.append("axis: \(axis.joined(separator: ", "))")
        }
        if let motor = response.diagnosis?.report?.motor_err_names, !motor.isEmpty {
            parts.append("motor: \(motor.joined(separator: ", "))")
        }
        if let encoder = response.diagnosis?.report?.enc_err_names, !encoder.isEmpty {
            parts.append("encoder: \(encoder.joined(separator: ", "))")
        }
        if let controller = response.diagnosis?.report?.ctrl_err_names, !controller.isEmpty {
            parts.append("controller: \(controller.joined(separator: ", "))")
        }
        if !parts.isEmpty {
            return parts.joined(separator: " | ")
        }
        if let snapshot = response.snapshot,
           (snapshot.axis_err ?? 0) != 0 || (snapshot.motor_err ?? 0) != 0 || (snapshot.enc_err ?? 0) != 0 || (snapshot.ctrl_err ?? 0) != 0 {
            return "axis=\(snapshot.axis_err ?? 0), motor=\(snapshot.motor_err ?? 0), encoder=\(snapshot.enc_err ?? 0), controller=\(snapshot.ctrl_err ?? 0)"
        }
        return nil
    }

    private func velocitySweepCommandValues() -> [Double]? {
        let rawStart = directControlForm.sweepStartTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines)
        let rawStop = directControlForm.sweepStopTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines)
        let rawStep = directControlForm.sweepStepTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let start = Double(rawStart),
              let stop = Double(rawStop),
              let stepValue = Double(rawStep),
              abs(stepValue) > 1e-9
        else {
            return nil
        }
        let step = abs(stepValue)
        var values: [Double] = []
        if start <= stop {
            var current = start
            while current <= stop + 1e-9 {
                values.append(current)
                current += step
                if values.count > 64 { break }
            }
        } else {
            var current = start
            while current >= stop - 1e-9 {
                values.append(current)
                current -= step
                if values.count > 64 { break }
            }
        }
        return values.isEmpty ? nil : values
    }

    private func velocitySweepTier(for quality: DirectRunQuality) -> VelocitySweepPoint.Tier {
        switch quality.verdict {
        case .good:
            return .usable
        case .partial, .signInverted:
            return .borderline
        case .sensorMismatch, .wrongDirection, .faulted:
            return .faulted
        case .stalled, .informational:
            return .belowFloor
        }
    }

    private func summarizeVelocitySweep(points: [VelocitySweepPoint], stoppedReason: String?) -> VelocitySweepSummary {
        let breakaway = points
            .filter { $0.tier == .borderline || $0.tier == .usable }
            .map { abs($0.commandTurnsPerSecond) }
            .min()
        let usable = points
            .filter { $0.tier == .usable }
            .map { abs($0.commandTurnsPerSecond) }
            .min()
        return VelocitySweepSummary(
            points: points,
            breakawayFloorTurnsPerSecond: breakaway,
            usableFloorTurnsPerSecond: usable,
            stoppedReason: stoppedReason,
            completedAt: Date()
        )
    }

    private func captureDirectRunBaseline(
        mode: String,
        requestedMotorTurns: Double? = nil,
        requestedTurnsPerSecond: Double? = nil,
        requestedDurationS: Double? = nil,
        relative: Bool = false,
        releaseAfter: Bool = false
    ) -> DirectRunBaseline {
        let snap = liveMonitor.snapshot ?? response?.snapshot
        let sensor = liveMonitor.outputSensor ?? response?.output_sensor
        return DirectRunBaseline(
            mode: mode,
            startedAt: Date(),
            startMotorTurns: snap?.pos_est,
            startOutputTurns: sensor?.output_turns,
            startOutputVelTurnsS: sensor?.output_vel_turns_s,
            startLagOutputTurns: sensor?.compliance_lag_output_turns,
            requestedMotorTurns: requestedMotorTurns,
            requestedTurnsPerSecond: requestedTurnsPerSecond,
            requestedDurationS: requestedDurationS,
            relative: relative,
            releaseAfter: releaseAfter
        )
    }

    private func buildDirectRunQuality(from result: BackendResponse, baseline: DirectRunBaseline) -> DirectRunQuality {
        let resultObject = result.result?.objectValue ?? [:]
        let gearRatio = resolvedGearRatio
        let errorSummary = responseErrorSummary(result)
        let hasFaults = (result.capabilities?.has_latched_errors == true) || errorSummary != nil

        let expectedMotorDelta: Double? = {
            if baseline.mode == "position" {
                if let start = resultObject["start_pos_turns"]?.numberValue,
                   let target = resultObject["target_turns"]?.numberValue {
                    return target - start
                }
                return resultObject["requested_turns"]?.numberValue ?? baseline.requestedMotorTurns
            }
            if let turnsPerSecond = resultObject["turns_per_second"]?.numberValue ?? baseline.requestedTurnsPerSecond,
               let duration = resultObject["duration_s"]?.numberValue ?? baseline.requestedDurationS {
                return turnsPerSecond * duration
            }
            return nil
        }()

        let actualMotorDelta = resultObject["delta_turns"]?.numberValue
        let furthestMotorDeltaTurns = resultObject["furthest_motor_delta_turns"]?.numberValue
        let maxAbsMotorDeltaTurns = resultObject["max_abs_motor_delta_turns"]?.numberValue
        let effectiveMotorDeltaMagnitude: Double? = {
            let finalMag = actualMotorDelta.map { abs($0) }
            let peakMag = maxAbsMotorDeltaTurns
            if let finalMag, let peakMag {
                return max(finalMag, peakMag)
            }
            return finalMag ?? peakMag
        }()
        let motorFollowFraction: Double? = {
            guard let expectedMotorDelta, abs(expectedMotorDelta) > 1e-6, let effectiveMotorDeltaMagnitude else { return nil }
            return abs(effectiveMotorDeltaMagnitude / expectedMotorDelta)
        }()
        let reachedTarget = resultObject["reached_target"]?.boolValue
        let peakMotorVelTurnsS = resultObject["peak_vel_turns_s"]?.numberValue

        let liveOrResultSensor = liveMonitor.outputSensor ?? result.output_sensor
        let endOutputTurns = resultObject["end_output_turns"]?.numberValue ?? liveOrResultSensor?.output_turns
        let endOutputVelTurnsS = resultObject["end_output_vel_turns_s"]?.numberValue ?? liveOrResultSensor?.output_vel_turns_s
        let peakOutputVelTurnsS = resultObject["peak_output_vel_turns_s"]?.numberValue
        let furthestOutputDeltaTurns = resultObject["furthest_output_delta_turns"]?.numberValue
        let maxAbsOutputDeltaTurns = resultObject["max_abs_output_delta_turns"]?.numberValue
        let endLagOutputTurns = resultObject["end_lag_output_turns"]?.numberValue ?? liveOrResultSensor?.compliance_lag_output_turns
        let actualOutputDelta: Double? = resultObject["output_delta_turns"]?.numberValue ?? {
            guard let start = baseline.startOutputTurns, let end = endOutputTurns else { return nil }
            return end - start
        }()
        let effectiveOutputDeltaMagnitude: Double? = {
            let finalMag = actualOutputDelta.map { abs($0) }
            let peakMag = maxAbsOutputDeltaTurns
            if let finalMag, let peakMag {
                return max(finalMag, peakMag)
            }
            return finalMag ?? peakMag
        }()
        let expectedOutputDelta = expectedMotorDelta.map { $0 / gearRatio }
        let expectedOutputFromActualMotorTurns = effectiveMotorDeltaMagnitude.map { $0 / gearRatio }
        let outputFollowFraction: Double? = {
            guard let expectedOutputDelta, let effectiveOutputDeltaMagnitude, abs(expectedOutputDelta) > 1e-6 else { return nil }
            return abs(effectiveOutputDeltaMagnitude / expectedOutputDelta)
        }()
        let transmissionFollowFraction: Double? = {
            guard let actualMotorDelta, abs(actualMotorDelta) > 1e-6, let effectiveOutputDeltaMagnitude else { return nil }
            let expectedFromActualMotor = abs(actualMotorDelta / gearRatio)
            guard expectedFromActualMotor > 1e-6 else { return nil }
            return effectiveOutputDeltaMagnitude / expectedFromActualMotor
        }()
        let directionCorrect: Bool? = {
            guard let expectedOutputDelta else { return nil }
            let signedObservedDelta: Double? = {
                if let furthestOutputDeltaTurns, let maxAbsOutputDeltaTurns, abs(furthestOutputDeltaTurns) >= maxAbsOutputDeltaTurns - 1e-9 {
                    return furthestOutputDeltaTurns
                }
                return actualOutputDelta
            }()
            guard let signedObservedDelta else { return nil }
            if abs(signedObservedDelta) < max(0.001, abs(expectedOutputDelta) * 0.10) {
                return nil
            }
            return expectedOutputDelta.sign == signedObservedDelta.sign
        }()
        let lagDeltaOutputTurns: Double? = resultObject["lag_delta_output_turns"]?.numberValue ?? {
            guard let start = baseline.startLagOutputTurns, let end = endLagOutputTurns else { return nil }
            return end - start
        }()
        let settledAtEnd: Bool? = {
            guard let endOutputVelTurnsS else { return nil }
            return abs(endOutputVelTurnsS) <= 0.01
        }()

        let verdict: DirectRunQuality.Verdict
        let headline: String
        let explanation: String
        let sensorMismatchDetected: Bool = {
            guard let expectedOutputFromActualMotorTurns,
                  abs(expectedOutputFromActualMotorTurns) >= 0.01,
                  let effectiveOutputDeltaMagnitude
            else { return false }
            return effectiveOutputDeltaMagnitude < max(0.0015, abs(expectedOutputFromActualMotorTurns) * 0.10)
        }()

        let signInversionDetected: Bool = {
            guard let directionCorrect, directionCorrect == false else { return false }
            guard let effectiveOutputDeltaMagnitude, effectiveOutputDeltaMagnitude >= 0.0015 else { return false }
            if let transmissionFollowFraction, transmissionFollowFraction >= 0.20 {
                return true
            }
            if let expectedOutputFromActualMotorTurns, abs(expectedOutputFromActualMotorTurns) >= 0.005 {
                return effectiveOutputDeltaMagnitude >= abs(expectedOutputFromActualMotorTurns) * 0.50
            }
            return false
        }()

        if hasFaults {
            verdict = .faulted
            headline = "Faulted"
            explanation = errorSummary ?? "The run ended with latched motor, encoder, or controller errors."
        } else if signInversionDetected {
            verdict = .signInverted
            headline = "Sign inverted"
            explanation = "The output moved with meaningful magnitude, but opposite sign from the motor convention. Treat this as an output-sensor sign convention issue, not a failed breakaway run."
        } else if baseline.mode == "velocity" && (baseline.requestedDurationS == nil || baseline.requestedDurationS == 0) {
            verdict = .informational
            headline = "Live velocity only"
            explanation = "Timed velocity runs are needed for a meaningful quality score. Use a duration to compare command and output motion."
        } else if let directionCorrect, directionCorrect == false {
            verdict = .wrongDirection
            headline = "Wrong direction"
            explanation = "The output moved opposite to the commanded direction."
        } else if sensorMismatchDetected {
            verdict = .sensorMismatch
            headline = "Sensor mismatch"
            explanation = "Motor excursion implies meaningful output travel, but the output sensor did not report it. Treat this as output-sensor or transmission mismatch, not a low-speed floor result."
        } else if let expectedOutputDelta, let effectiveOutputDeltaMagnitude,
                  effectiveOutputDeltaMagnitude < max(0.001, abs(expectedOutputDelta) * 0.20) {
            verdict = .stalled
            headline = "Weak or stalled"
            explanation = "The command produced too little real output excursion to count as a healthy run."
        } else if let outputFollowFraction, outputFollowFraction >= 0.60,
                  (directionCorrect ?? true),
                  (settledAtEnd ?? true) {
            verdict = .good
            headline = "Clean enough"
            explanation = "The output followed the command in the correct direction with bounded lag and a calm stop."
        } else if (directionCorrect ?? true) {
            verdict = .partial
            headline = "Partial follow"
            explanation = "The output moved in the correct direction, but follow quality or stop quality was still weak."
        } else {
            verdict = .informational
            headline = "Incomplete data"
            explanation = "The run completed, but there was not enough output-side data to score it confidently."
        }

        return DirectRunQuality(
            mode: baseline.mode,
            verdict: verdict,
            headline: headline,
            explanation: explanation,
            expectedMotorDeltaTurns: expectedMotorDelta,
            actualMotorDeltaTurns: actualMotorDelta,
            furthestMotorDeltaTurns: furthestMotorDeltaTurns,
            maxAbsMotorDeltaTurns: maxAbsMotorDeltaTurns,
            motorFollowFraction: motorFollowFraction,
            expectedOutputDeltaTurns: expectedOutputDelta,
            expectedOutputFromActualMotorTurns: expectedOutputFromActualMotorTurns,
            actualOutputDeltaTurns: actualOutputDelta,
            furthestOutputDeltaTurns: furthestOutputDeltaTurns,
            maxAbsOutputDeltaTurns: maxAbsOutputDeltaTurns,
            outputFollowFraction: outputFollowFraction,
            transmissionFollowFraction: transmissionFollowFraction,
            directionCorrect: directionCorrect,
            motorReachedTarget: reachedTarget,
            finalOutputVelTurnsS: endOutputVelTurnsS,
            lagDeltaOutputTurns: lagDeltaOutputTurns,
            peakMotorVelTurnsS: peakMotorVelTurnsS,
            peakOutputVelTurnsS: peakOutputVelTurnsS,
            errorSummary: errorSummary,
            assistUsed: resultObject["assist_used"]?.boolValue ?? false,
            assistFloorTurnsPerSecond: resultObject["assist_floor_turns_s"]?.numberValue,
            assistKickTurnsPerSecond: resultObject["kick_turns_per_second"]?.numberValue,
            assistKickDurationS: resultObject["kick_duration_s_actual"]?.numberValue,
            assistBreakawayDetected: resultObject["breakaway_detected"]?.boolValue,
            assistBreakawayDetectedAtS: resultObject["breakaway_detected_at_s"]?.numberValue
        )
    }

    func pollTelemetryOnce() async {
        guard !telemetryRequestActive else { return }
        telemetryRequestActive = true
        defer { telemetryRequestActive = false }
        do {
            let result = try await backend.run(action: "telemetry", arguments: [], context: requestContext())
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
        let trimmedSpeedScale = moveForm.runtimeSpeedScale.trimmingCharacters(in: .whitespacesAndNewlines)
        if selectedProfileSupportsRuntimeSpeedTweak,
           let speedScale = Double(trimmedSpeedScale),
           speedScale > 0,
           abs(speedScale - 1.0) > 0.000001 {
            args.append(contentsOf: ["--speed-scale", String(format: "%.6g", speedScale)])
        }
        await run(action: "move-continuous-async", arguments: args, countsAsBlocking: false)
    }

    func commandDirectPosition() async {
        let startedTemporaryStream = !streamingStarted
        if startedTemporaryStream {
            await ensureStreaming()
        }
        let requestedTurns = Double(directControlForm.turns.trimmingCharacters(in: .whitespacesAndNewlines))
        let baseline = captureDirectRunBaseline(
            mode: "position",
            requestedMotorTurns: requestedTurns,
            relative: directControlForm.relativeTurns,
            releaseAfter: directControlForm.releaseAfterPosition
        )
        activeDirectRunBaseline = baseline
        var args = [
            "--turns", directControlForm.turns,
            "--target-tolerance-turns", directControlForm.targetToleranceTurns,
            "--target-vel-tolerance-turns-s", directControlForm.targetVelToleranceTurnsS,
            "--gear-ratio", moveForm.gearRatio,
        ]
        if directControlForm.relativeTurns {
            args.append("--relative")
        }
        if !directControlForm.timeoutSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            args.append(contentsOf: ["--timeout-s", directControlForm.timeoutSeconds])
        }
        if directControlForm.releaseAfterPosition {
            args.append("--release-after-command")
        }
        let result = await run(action: "command-position", arguments: args)
        if let result {
            latestDirectRunQuality = buildDirectRunQuality(from: result, baseline: baseline)
        }
        activeDirectRunBaseline = nil
        if startedTemporaryStream && !telemetryAutoRefresh {
            await disableStreamingIfAllowed(force: true)
        }
    }

    func commandDirectVelocity() async {
        let startedTemporaryStream = !streamingStarted
        if startedTemporaryStream {
            await ensureStreaming()
        }
        let hasTimedDuration = !directControlForm.durationSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
        let requestedTurnsPerSecond = Double(directControlForm.turnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines))
        let requestedDuration = hasTimedDuration
            ? Double(directControlForm.durationSeconds.trimmingCharacters(in: .whitespacesAndNewlines))
            : nil
        let baseline = captureDirectRunBaseline(
            mode: "velocity",
            requestedTurnsPerSecond: requestedTurnsPerSecond,
            requestedDurationS: requestedDuration,
            releaseAfter: directControlForm.releaseAfterVelocity
        )
        activeDirectRunBaseline = baseline
        var args = [
            "--turns-per-second", directControlForm.turnsPerSecond,
            "--gear-ratio", moveForm.gearRatio,
        ]
        if hasTimedDuration {
            args.append(contentsOf: ["--duration-s", directControlForm.durationSeconds])
        }
        if directControlForm.releaseAfterVelocity {
            args.append("--release-after-command")
        }
        let result = await run(action: "command-velocity", arguments: args)
        if let result {
            latestDirectRunQuality = buildDirectRunQuality(from: result, baseline: baseline)
        }
        activeDirectRunBaseline = nil
        if startedTemporaryStream && !telemetryAutoRefresh && hasTimedDuration {
            await disableStreamingIfAllowed(force: true)
        }
    }

    func runVelocityBreakawaySweep() async {
        guard let velocities = velocitySweepCommandValues(),
              let duration = Double(directControlForm.sweepDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines)),
              duration > 0.0
        else {
            lastClientError = "Sweep start/stop/step/duration must be valid numbers."
            return
        }
        let startedTemporaryStream = !streamingStarted
        if startedTemporaryStream {
            await ensureStreaming()
        }
        blockingActionInFlight = true
        defer {
            blockingActionInFlight = false
            activeDirectRunBaseline = nil
        }
        lastClientError = nil
        latestVelocitySweep = nil

        var points: [VelocitySweepPoint] = []
        var stoppedReason: String?
        for velocity in velocities {
            let baseline = captureDirectRunBaseline(
                mode: "velocity",
                requestedTurnsPerSecond: velocity,
                requestedDurationS: duration,
                releaseAfter: true
            )
            activeDirectRunBaseline = baseline
            let result = await run(
                action: "command-velocity",
                arguments: [
                    "--turns-per-second", String(velocity),
                    "--duration-s", String(duration),
                    "--release-after-command",
                    "--gear-ratio", moveForm.gearRatio,
                ],
                countsAsBlocking: false
            )
            activeDirectRunBaseline = nil
            guard let result else {
                stoppedReason = lastClientError ?? "Backend request failed during sweep."
                break
            }
            let quality = buildDirectRunQuality(from: result, baseline: baseline)
            latestDirectRunQuality = quality
            let point = VelocitySweepPoint(
                commandTurnsPerSecond: velocity,
                quality: quality,
                tier: velocitySweepTier(for: quality)
            )
            points.append(point)
            if point.tier == .faulted {
                stoppedReason = quality.errorSummary ?? quality.headline
                break
            }
        }
        latestVelocitySweep = summarizeVelocitySweep(points: points, stoppedReason: stoppedReason)
        if startedTemporaryStream && !telemetryAutoRefresh {
            await disableStreamingIfAllowed(force: true)
        }
    }

    func commandAssistedVelocity() async {
        guard let assistFloorTurnsPerSecond else {
            lastClientError = "Run sweep first or set a manual floor override."
            return
        }
        let startedTemporaryStream = !streamingStarted
        if startedTemporaryStream {
            await ensureStreaming()
        }
        let hasTimedDuration = !directControlForm.durationSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
        let requestedTurnsPerSecond = Double(directControlForm.turnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines))
        let requestedDuration = hasTimedDuration
            ? Double(directControlForm.durationSeconds.trimmingCharacters(in: .whitespacesAndNewlines))
            : nil
        let baseline = captureDirectRunBaseline(
            mode: "velocity assist",
            requestedTurnsPerSecond: requestedTurnsPerSecond,
            requestedDurationS: requestedDuration,
            releaseAfter: directControlForm.releaseAfterVelocity
        )
        activeDirectRunBaseline = baseline
        var args = [
            "--turns-per-second", directControlForm.turnsPerSecond,
            "--duration-s", directControlForm.durationSeconds,
            "--gear-ratio", moveForm.gearRatio,
            "--breakaway-floor-turns-s", String(assistFloorTurnsPerSecond),
            "--kick-max-duration-s", directControlForm.assistKickMaxDurationSeconds,
            "--breakaway-output-turns", directControlForm.assistBreakawayOutputTurns,
        ]
        if directControlForm.releaseAfterVelocity {
            args.append("--release-after-command")
        }
        let result = await run(action: "command-velocity-assist", arguments: args)
        if let result {
            latestDirectRunQuality = buildDirectRunQuality(from: result, baseline: baseline)
        }
        activeDirectRunBaseline = nil
        if startedTemporaryStream && !telemetryAutoRefresh {
            await disableStreamingIfAllowed(force: true)
        }
    }

    func requestAxisState(_ state: Int, waitIdle: Bool = false, waitState: Bool = false, timeoutSeconds: Double? = nil, clearFirst: Bool = false) async {
        var args = ["--requested-state", String(state)]
        if waitIdle {
            args.append("--wait-idle")
        }
        if waitState {
            args.append("--wait-state")
        }
        if clearFirst {
            args.append("--clear-first")
        }
        if let timeoutSeconds {
            args.append(contentsOf: ["--timeout-s", String(format: "%.6g", timeoutSeconds)])
        }
        await run(action: "set-requested-state", arguments: args)
    }

    func refreshSyncAxesStatus() async {
        async let axisA: Void = runForAxis(
            axisIndex: syncMoveForm.axisAIndex,
            deviceSerial: syncMoveForm.serialA,
            action: "status",
            storeInto: .syncAxisA
        )
        async let axisB: Void = runForAxis(
            axisIndex: syncMoveForm.axisBIndex,
            deviceSerial: syncMoveForm.serialB,
            action: "status",
            storeInto: .syncAxisB
        )
        _ = await (axisA, axisB)
    }

    func discoverSingleAxisBoards() async {
        do {
            let result = try await backend.run(action: "discover-boards", arguments: [], context: requestContext())
            response = result
            syncResultResponse = result
            let serials = discoveredBoardSerials(from: result)
            detectedBoardSerials = serials
            if let first = serials.first, selectedBoardSerial.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                selectedBoardSerial = first
            } else if !selectedBoardSerial.isEmpty, !serials.contains(selectedBoardSerial) {
                selectedBoardSerial = serials.first ?? ""
            }
            lastClientError = serials.isEmpty ? "No ODrive runtime boards discovered." : nil
            if !serials.isEmpty {
                await singleAxisContextChanged()
            }
        } catch {
            lastClientError = error.localizedDescription
        }
    }

    func singleAxisContextChanged() async {
        await disableStreamingIfAllowed(force: true)
        liveMonitor.reset()
        response = nil
        autoDirectionResponse = nil
        guidedBringupResponse = nil
        motorDirectionDirty = false
        await refreshStatus()
    }

    func discoverAndFillSyncBoardSerials() async {
        do {
            let result = try await backend.run(action: "discover-boards", arguments: [], context: requestContext())
            response = result
            syncResultResponse = result
            let serials = discoveredBoardSerials(from: result)
            detectedBoardSerials = serials
            if serials.isEmpty {
                lastClientError = "No ODrive runtime boards discovered."
                return
            }
            if serials.count == 1 {
                syncMoveForm.serialA = serials[0]
                syncMoveForm.serialB = serials[0]
            } else {
                syncMoveForm.serialA = serials[0]
                syncMoveForm.serialB = serials[1]
            }
            lastClientError = nil
        } catch {
            lastClientError = error.localizedDescription
        }
    }

    func captureSyncZero(axisRole: String) async {
        let isA = axisRole.lowercased() == "a"
        let targetAxisIndex = isA ? syncMoveForm.axisAIndex : syncMoveForm.axisBIndex
        do {
            let result = try await backend.run(
                action: "status",
                arguments: [],
                context: requestContext(
                    axisIndex: targetAxisIndex,
                    deviceSerial: isA ? syncMoveForm.serialA : syncMoveForm.serialB
                )
            )
            if isA {
                syncAxisAResponse = result
                if let pos = result.snapshot?.pos_est {
                    syncMoveForm.zeroATurnsMotor = String(format: "%.6f", pos)
                }
            } else {
                syncAxisBResponse = result
                if let pos = result.snapshot?.pos_est {
                    syncMoveForm.zeroBTurnsMotor = String(format: "%.6f", pos)
                }
            }
            mergeProfilesIfNeeded(from: result)
        } catch {
            lastClientError = error.localizedDescription
        }
    }

    func moveTwoAxesSynced() async {
        if syncAxisTargetsConflict {
            lastClientError = "Axis A and Axis B currently resolve to the same board/axis."
            return
        }
        lastClientError = nil
        blockingActionInFlight = true
        defer { blockingActionInFlight = false }
        var args = [
            "--axis-a-index", String(syncMoveForm.axisAIndex),
            "--axis-b-index", String(syncMoveForm.axisBIndex),
            "--angle-a-deg", syncMoveForm.angleADeg,
            "--angle-b-deg", syncMoveForm.angleBDeg,
            "--angle-space", syncMoveForm.angleSpace,
            "--gear-ratio-a", syncMoveForm.gearRatioA,
            "--gear-ratio-b", syncMoveForm.gearRatioB,
            "--profile-name", syncMoveForm.profileName,
            "--profile-a-name", syncMoveForm.profileAName,
            "--profile-b-name", syncMoveForm.profileBName,
        ]
        if !syncMoveForm.serialA.trimmingCharacters(in: .whitespaces).isEmpty {
            args.append(contentsOf: ["--serial-a", syncMoveForm.serialA])
        }
        if !syncMoveForm.serialB.trimmingCharacters(in: .whitespaces).isEmpty {
            args.append(contentsOf: ["--serial-b", syncMoveForm.serialB])
        }
        if !syncMoveForm.zeroATurnsMotor.trimmingCharacters(in: .whitespaces).isEmpty {
            args.append(contentsOf: ["--zero-a-turns-motor", syncMoveForm.zeroATurnsMotor])
        }
        if !syncMoveForm.zeroBTurnsMotor.trimmingCharacters(in: .whitespaces).isEmpty {
            args.append(contentsOf: ["--zero-b-turns-motor", syncMoveForm.zeroBTurnsMotor])
        }
        if !syncMoveForm.timeoutSeconds.trimmingCharacters(in: .whitespaces).isEmpty {
            args.append(contentsOf: ["--timeout-s", syncMoveForm.timeoutSeconds])
        }
        if syncMoveForm.releaseAfterMove {
            args.append("--release-after-move")
        }
        do {
            let result = try await backend.run(
                action: "move-two-axes-synced",
                arguments: args,
                context: requestContext(axisIndex: syncMoveForm.axisAIndex, deviceSerial: syncMoveForm.serialA)
            )
            syncResultResponse = result
            response = result
            mergeProfilesIfNeeded(from: result)
            await refreshSyncAxesStatus()
        } catch {
            lastClientError = error.localizedDescription
        }
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

    func forkLoadedProfileEditor() {
        var forked = profileEditor
        forked.forkForEditing()
        profileEditor = forked
    }

    @discardableResult
    private func run(
        action: String,
        arguments: [String] = [],
        countsAsBlocking: Bool = true,
        storePrimaryResponse: Bool = true,
        storeTelemetryResponse: Bool = true
    ) async -> BackendResponse? {
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
            return result
        } catch {
            lastClientError = error.localizedDescription
            return nil
        }
    }

    private func ensureStreaming() async {
        guard !streamingStarted else { return }
        do {
            let stream = try await backend.ensureEventStream(context: requestContext(), intervalMs: telemetryStreamRate.intervalMs)
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
            mergeLiveStatusIfNeeded(from: event)
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
        if moveForm.profileName == Self.legacyDefaultProfileName, available.contains(Self.preferredMksStartupProfileName) {
            moveForm.profileName = Self.preferredMksStartupProfileName
        }
        if available.contains(moveForm.profileName) == false, let first = available.first {
            moveForm.profileName = first
        }
        if syncMoveForm.profileName.isEmpty, let first = available.first {
            syncMoveForm.profileName = first
        }
        if syncMoveForm.profileName == Self.legacyDefaultProfileName, available.contains(Self.preferredMksStartupProfileName) {
            syncMoveForm.profileName = Self.preferredMksStartupProfileName
        }
        if available.contains(syncMoveForm.profileName) == false, let first = available.first {
            syncMoveForm.profileName = first
        }
        if syncMoveForm.profileAName.isEmpty {
            syncMoveForm.profileAName = syncMoveForm.profileName
        }
        if syncMoveForm.profileAName == Self.legacyDefaultProfileName, available.contains(Self.preferredMksStartupProfileName) {
            syncMoveForm.profileAName = Self.preferredMksStartupProfileName
        }
        if available.contains(syncMoveForm.profileAName) == false, let first = available.first {
            syncMoveForm.profileAName = first
        }
        if syncMoveForm.profileBName.isEmpty {
            syncMoveForm.profileBName = syncMoveForm.profileName
        }
        if syncMoveForm.profileBName == Self.legacyDefaultProfileName, available.contains(Self.preferredMksStartupProfileName) {
            syncMoveForm.profileBName = Self.preferredMksStartupProfileName
        }
        if available.contains(syncMoveForm.profileBName) == false, let first = available.first {
            syncMoveForm.profileBName = first
        }
    }

    private func mergeProfileEditorIfNeeded(from result: BackendResponse) {
        guard let editor = result.profile_editor else { return }
        profileEditor = ProfileEditorFormState(editor: editor)
        let moveMode = (editor.move_mode ?? "").trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
        if moveMode != "mks_directional_slew_direct" && moveMode != "mks_directional_velocity_travel_direct" {
            moveForm.runtimeSpeedScale = "1.0"
        }
        if moveForm.profileName != editor.name {
            moveForm.profileName = editor.name
        }
        if syncMoveForm.profileName.isEmpty {
            syncMoveForm.profileName = editor.name
        }
    }

    private func mergeLiveStatusIfNeeded(from result: BackendResponse) {
        liveMonitor.merge(from: result)
        if let motorDirection = result.snapshot?.motor_direction {
            if !motorDirectionDirty || motorDirection == motorDirectionSelection {
                motorDirectionSelection = motorDirection
                motorDirectionDirty = false
            }
        }
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
        let trimmedSpeedScale = moveForm.runtimeSpeedScale.trimmingCharacters(in: .whitespacesAndNewlines)
        if selectedProfileSupportsRuntimeSpeedTweak,
           let speedScale = Double(trimmedSpeedScale),
           speedScale > 0,
           abs(speedScale - 1.0) > 0.000001 {
            args.append(contentsOf: ["--speed-scale", String(format: "%.6g", speedScale)])
        }
        await run(
            action: "move-continuous-async",
            arguments: args,
            countsAsBlocking: false,
            storePrimaryResponse: true,
            storeTelemetryResponse: true
        )
    }

    private enum AxisResponseTarget {
        case syncAxisA
        case syncAxisB
    }

    private func runForAxis(
        axisIndex: Int,
        deviceSerial: String = "",
        action: String,
        arguments: [String] = [],
        storeInto target: AxisResponseTarget
    ) async {
        do {
            let result = try await backend.run(
                action: action,
                arguments: arguments,
                context: requestContext(axisIndex: axisIndex, deviceSerial: deviceSerial)
            )
            switch target {
            case .syncAxisA:
                syncAxisAResponse = result
            case .syncAxisB:
                syncAxisBResponse = result
            }
            mergeProfilesIfNeeded(from: result)
        } catch {
            lastClientError = error.localizedDescription
        }
    }

    private func discoveredBoardSerials(from result: BackendResponse) -> [String] {
        guard
            let object = result.result?.objectValue,
            let devices = object["devices"]?.arrayValue
        else {
            return []
        }
        return devices.compactMap { device in
            device.objectValue?["serial_number"]?.stringValue
        }
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
    let visiblePanels: Set<String>
    let togglePanel: (ConsolePanel) -> Void
    let showAllPanels: () -> Void

    var body: some View {
        ScrollView {
            Form {
                Section("Visible Panels") {
                    VStack(alignment: .leading, spacing: 8) {
                        Text("Choose which main panels stay on screen. This is persisted, so the app reopens with the same layout next time.")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                        ForEach(ConsolePanel.allCases) { panel in
                            Button {
                                togglePanel(panel)
                            } label: {
                                HStack(alignment: .top, spacing: 10) {
                                    Image(systemName: visiblePanels.contains(panel.rawValue) ? "checkmark.circle.fill" : "circle")
                                        .foregroundStyle(visiblePanels.contains(panel.rawValue) ? Color.accentColor : Color.secondary)
                                    VStack(alignment: .leading, spacing: 2) {
                                        Text(panel.title)
                                            .foregroundStyle(.primary)
                                        Text(panel.subtitle)
                                            .font(.caption)
                                            .foregroundStyle(.secondary)
                                    }
                                    Spacer()
                                }
                                .contentShape(Rectangle())
                            }
                            .buttonStyle(.plain)
                        }
                        Button("Show All Panels") {
                            showAllPanels()
                        }
                        .buttonStyle(.borderless)
                        .font(.caption)
                    }
                    .padding(.vertical, 4)
                }

                Section("Backend") {
                    LabeledInputField(title: "Repo root", text: $vm.repoRoot)
                    Stepper("Axis \(vm.axisIndex)", value: $vm.axisIndex, in: 0...1)
                    HStack {
                        Text("Board target")
                        Spacer()
                        Button("Detect Boards") {
                            Task { await vm.discoverSingleAxisBoards() }
                        }
                        .disabled(vm.isBusy)
                        Button("Apply Target") {
                            Task { await vm.singleAxisContextChanged() }
                        }
                        .disabled(vm.isBusy)
                    }
                    if !vm.detectedBoardSerials.isEmpty {
                        Picker("Detected board", selection: $vm.selectedBoardSerial) {
                            Text("Auto / first found").tag("")
                            ForEach(vm.detectedBoardSerials, id: \.self) { serial in
                                Text(serial).tag(serial)
                            }
                        }
                    }
                    LabeledInputField(title: "Board serial (optional)", text: $vm.selectedBoardSerial)
                    LabeledInputField(title: "KV estimate", text: $vm.kvEstimate)
                    LabeledInputField(title: "Line-line resistance (ohm)", text: $vm.lineLineROhm)
                    LabeledInputField(title: "Clear-errors settle (s)", text: $vm.settleSeconds)
                    Toggle("Debug backend", isOn: $vm.debugMode)
                }

                Section("Motor Direction") {
                    VStack(alignment: .leading, spacing: 8) {
                        Toggle(
                            "Reverse motor direction",
                            isOn: Binding(
                                get: { vm.motorDirectionSelection < 0 },
                                set: { vm.setMotorDirectionSelection($0 ? -1 : 1) }
                            )
                        )
                        Text(
                            "Current: \(vm.currentMotorDirection.map { String(format: "%+d", $0) } ?? "unknown") · applies to runtime config only, not saved to flash."
                        )
                        .font(.caption)
                        .foregroundStyle(.secondary)
                        HStack {
                            Button("Auto Detect Direction") {
                                Task { await vm.autoDetectMotorDirection() }
                            }
                            .disabled(
                                vm.isBusy
                                || vm.capabilities?.motion_active == true
                                || vm.capabilities?.startup_ready == false
                            )
                            Button("Apply Motor Direction") {
                                Task { await vm.applyMotorDirection() }
                            }
                            .disabled(
                                vm.isBusy
                                || vm.capabilities?.motion_active == true
                                || vm.currentMotorDirection == vm.motorDirectionSelection
                            )
                        }
                        Text("Auto detect runs the existing low-stress sign-consistency contract on the currently selected board and axis. Use it only after startup is ready.")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                        if let result = vm.autoDirectionResponse {
                            AutoDirectionResultView(response: result)
                        }
                    }
                }

                Section("Actions") {
                    VStack(alignment: .leading, spacing: 12) {
                        Text("Refresh / Inspect")
                            .font(.caption.weight(.semibold))
                            .foregroundStyle(.secondary)
                        HStack {
                            Button("Refresh Status") { Task { await vm.refreshStatus() } }
                            Button("Diagnose") { Task { await vm.runDiagnose() } }
                            Button("Motor Fact Sheet") { Task { await vm.runFactSheet() } }
                        }

                        Text("Calibration States")
                            .font(.caption.weight(.semibold))
                            .foregroundStyle(.secondary)
                        HStack {
                            Button("Full Cal (3)") { Task { await vm.startup() } }
                                .disabled(vm.isBusy || vm.capabilities?.can_startup == false)
                            Button("Motor Cal (4)") {
                                Task { await vm.requestAxisState(4, waitIdle: true, timeoutSeconds: 20, clearFirst: true) }
                            }
                            .disabled(vm.isBusy)
                            Button("Enc Offset (7)") {
                                Task { await vm.requestAxisState(7, waitIdle: true, timeoutSeconds: 20, clearFirst: true) }
                            }
                            .disabled(vm.isBusy)
                        }

                        Text("Control States")
                            .font(.caption.weight(.semibold))
                            .foregroundStyle(.secondary)
                        HStack {
                            Button("Closed Loop (8)") {
                                Task { await vm.requestAxisState(8, waitState: true, timeoutSeconds: 3.0) }
                            }
                            .disabled(vm.isBusy)
                            Button("Idle (1)") { Task { await vm.idle() } }
                                .disabled(vm.isBusy || vm.capabilities?.can_idle == false)
                            Button("Clear Errors") { Task { await vm.clearErrors() } }
                                .disabled(vm.isBusy || vm.capabilities?.can_clear_errors == false)
                        }
                    }
                }
            }
            .formStyle(.grouped)
        }
        .frame(minWidth: 300, idealWidth: 340, maxWidth: 380)
        .onChange(of: vm.axisIndex) { _, _ in
            Task { await vm.singleAxisContextChanged() }
        }
    }
}

struct AutoDirectionResultView: View {
    let response: BackendResponse

    private var resultObject: [String: JSONValue]? {
        response.result?.objectValue
    }

    private var selectedDirection: Int? {
        resultObject?["selected_direction"]?.numberValue.map(Int.init)
    }

    private var trustResult: Bool? {
        resultObject?["trust_result"]?.boolValue
    }

    private var confidence: Double? {
        resultObject?["confidence"]?.numberValue
    }

    private var winnerClassification: String? {
        resultObject?["winner_classification"]?.stringValue
    }

    private var validationClassification: String? {
        resultObject?["validation_classification"]?.stringValue
    }

    private var runnerUpMargin: Double? {
        resultObject?["runner_up_score_margin"]?.numberValue
    }

    private var trustReason: String? {
        resultObject?["trust_reason"]?.stringValue
    }

    private var persistError: String? {
        resultObject?["persist_error"]?.stringValue
    }

    private func confidenceLabel(_ value: Double?) -> String {
        guard let value else { return "unknown" }
        if value >= 0.75 { return String(format: "%.2f (high)", value) }
        if value >= 0.45 { return String(format: "%.2f (medium)", value) }
        return String(format: "%.2f (low)", value)
    }

    private var trustColor: Color {
        trustResult == true ? .green : .red
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            Divider()
            Text("Last Auto Detect Result")
                .font(.subheadline.weight(.semibold))
            HStack(spacing: 12) {
                StatusBadge(
                    title: "Selected",
                    value: selectedDirection.map { String(format: "%+d", $0) } ?? "unknown",
                    color: .blue
                )
                StatusBadge(
                    title: "Trust",
                    value: (trustResult == true) ? "trusted" : "not trusted",
                    color: trustColor
                )
                StatusBadge(
                    title: "Confidence",
                    value: confidenceLabel(confidence),
                    color: (confidence ?? 0.0) >= 0.45 ? .green : .orange
                )
                StatusBadge(
                    title: "Validation",
                    value: validationClassification ?? "unknown",
                    color: trustColor
                )
            }
            if let winnerClassification, !winnerClassification.isEmpty {
                Text("Winner classification: \(winnerClassification)")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            if let runnerUpMargin {
                Text(String(format: "Runner-up score margin: %.1f", runnerUpMargin))
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            if let trustReason, !trustReason.isEmpty {
                Text(trustReason)
                    .font(.caption)
            }
            if let persistError, !persistError.isEmpty {
                Text("Persist error: \(persistError)")
                    .font(.caption)
                    .foregroundStyle(.red)
            }
        }
        .padding(.top, 4)
    }
}

struct TelemetrySectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @ObservedObject var live: LiveMonitorModel

    private func directionVerdict(for sample: TelemetrySample) -> (label: String, color: Color) {
        let err = sample.trackingError
        let vel = sample.velEst
        if abs(err) <= 0.002 && abs(vel) <= 0.02 {
            return ("holding", .gray)
        }
        if abs(vel) <= 0.02 {
            return ("not moving", .orange)
        }
        if (err * vel) > 0 {
            return ("moving toward target", .green)
        }
        if (err * vel) < 0 {
            return ("moving away from target", .red)
        }
        return ("indeterminate", .orange)
    }

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
            }

            HStack {
                Picker("Chart rate", selection: $vm.telemetryStreamRate) {
                    ForEach(TelemetryStreamRate.allCases) { rate in
                        Text(rate.rawValue).tag(rate)
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

                Button("Clear") {
                    live.clearTelemetryHistory()
                }
                .disabled(live.telemetrySamples.isEmpty)
            }
            .onChange(of: vm.telemetryStreamRate) { _, newValue in
                vm.telemetryStreamRateChanged(newValue)
            }

            if live.telemetrySamples.isEmpty {
                Text("No telemetry samples yet.")
                    .foregroundStyle(.secondary)
            } else {
                TelemetryGraphView(samples: live.telemetrySamples, metric: vm.telemetryMetric)
                    .frame(minHeight: 220)
            }

            if let latest = live.telemetrySamples.last {
                let verdict = directionVerdict(for: latest)
                VStack(alignment: .leading, spacing: 10) {
                    HStack(spacing: 12) {
                        StatusBadge(
                            title: "Direction Check",
                            value: verdict.label,
                            color: verdict.color
                        )
                        StatusBadge(
                            title: "Tracking Error",
                            value: String(format: "%+.4f t", latest.trackingError),
                            color: latest.trackingError == 0 ? .gray : (latest.trackingError > 0 ? .green : .orange)
                        )
                        StatusBadge(
                            title: "Velocity",
                            value: String(format: "%+.4f t/s", latest.velEst),
                            color: latest.velEst == 0 ? .gray : (latest.velEst > 0 ? .green : .orange)
                        )
                    }
                    Text("Rule: error and velocity with the same sign means the axis is moving toward the commanded target. Opposite sign means it is moving away.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }

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
    @ObservedObject var vm: OperatorConsoleViewModel
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
            HStack(alignment: .top, spacing: 12) {
                VStack(alignment: .leading, spacing: 4) {
                    Text("State at a Glance")
                        .font(.title2.bold())
                    Text(vm.stateRepairHint)
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
                Spacer()
                Button(vm.stateRepairButtonTitle) {
                    Task { await vm.repairAxisState() }
                }
                .buttonStyle(.borderedProminent)
                .disabled(vm.isBusy || vm.capabilities?.motion_active == true)
            }
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
            if let errorSummary = vm.readinessErrorSummary {
                Text("Current errors: \(errorSummary)")
                    .font(.caption)
                    .foregroundStyle(.red)
            }
        }
    }
}

struct OutputSensorSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    private var sensor: BackendOutputSensor? { vm.outputSensor }

    private func statusColor(_ value: Bool?) -> Color {
        value == true ? .green : .orange
    }

    private func formatted(_ value: Double?, suffix: String) -> String {
        guard let value else { return "unknown" }
        return String(format: "%.6f %@", value, suffix)
    }

    private func compact(_ value: Double?, suffix: String) -> String {
        guard let value else { return "unknown" }
        return String(format: "%.4f %@", value, suffix)
    }

    var body: some View {
        guard let sensor else {
            return AnyView(EmptyView())
        }
        return AnyView(
            VStack(alignment: .leading, spacing: 12) {
                HStack {
                    VStack(alignment: .leading, spacing: 4) {
                        Text("Output Sensor")
                            .font(.title2.bold())
                        Text("Read-only external output-angle bridge. This does not replace the MKS encoder path; it gives the app the real gearbox-output measurement for diagnostics and later outer-loop work.")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                    Spacer()
                    if let port = sensor.port {
                        Text(port)
                            .font(.system(.caption, design: .monospaced))
                            .foregroundStyle(.secondary)
                    }
                }

                LazyVGrid(columns: [GridItem(.adaptive(minimum: 220), spacing: 12)], spacing: 12) {
                    StatusBadge(title: "Bridge", value: (sensor.connected == true) ? "connected" : "not connected", color: statusColor(sensor.connected))
                    StatusBadge(title: "Sensor Health", value: (sensor.healthy == true) ? "healthy" : "degraded", color: statusColor(sensor.healthy))
                    StatusBadge(title: "Streaming", value: (sensor.streaming == true) ? "on" : "off", color: statusColor(sensor.streaming))
                    StatusBadge(title: "Homed", value: (sensor.homed == true) ? "yes" : "no", color: statusColor(sensor.homed))
                    StatusBadge(title: "Encoder", value: sensor.encoder_name ?? "unknown", color: .gray)
                    StatusBadge(title: "Output Sign", value: sensor.output_sign.map { $0 < 0 ? "-1" : "+1" } ?? "unknown", color: .gray)
                    StatusBadge(title: "Output Angle", value: compact(sensor.output_turns, suffix: "t"), color: .gray)
                    StatusBadge(title: "Output Velocity", value: compact(sensor.output_vel_turns_s, suffix: "t/s"), color: .gray)
                    StatusBadge(title: "Lag", value: compact(sensor.compliance_lag_output_turns, suffix: "out t"), color: .orange)
                }

                HStack(spacing: 18) {
                    Text("raw \(sensor.raw_angle_counts.map(String.init) ?? "unknown")")
                    Text("sample age \(sensor.last_sample_age_s.map { String(format: "%.3f s", $0) } ?? "unknown")")
                    Text("mag 0x\(sensor.mag_status_bits.map { String($0, radix: 16, uppercase: true) } ?? "0")")
                    Text("diag 0x\(sensor.diag_bits.map { String($0, radix: 16, uppercase: true) } ?? "0")")
                }
                .font(.system(.caption, design: .monospaced))
                .foregroundStyle(.secondary)

                if let lagMotor = sensor.compliance_lag_turns, let lagOut = sensor.compliance_lag_output_turns {
                    Text("Motor-output lag: \(formatted(lagMotor, suffix: "motor t")) / \(formatted(lagOut, suffix: "output t"))")
                        .font(.system(.caption, design: .monospaced))
                        .foregroundStyle(.secondary)
                }

                if let lastError = sensor.last_error, !lastError.isEmpty {
                    Text(lastError)
                        .font(.caption)
                        .foregroundStyle(.red)
                }
            }
            .padding(16)
            .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
        )
    }
}

struct OutputSensorLaunchWarningView: View {
    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack {
                Text("Output Sensor Not Enabled")
                    .font(.headline)
                Spacer()
                StatusBadge(title: "Bridge Env", value: "missing", color: .orange)
            }
            Text("This FOCUI process was launched without `ROBOT_OUTPUT_SENSOR_PORT`, so the external output-sensor bridge is disabled and the Output Sensor card will not appear.")
                .font(.caption)
                .foregroundStyle(.secondary)
            Text("Where the card appears: directly below `State at a Glance` and above `Guided Bring-Up`.")
                .font(.caption)
                .foregroundStyle(.secondary)
            Text("Launch FOCUI from the same shell where you exported `ROBOT_OUTPUT_SENSOR_PORT`, or add the output-sensor env vars to the Xcode Run scheme.")
                .font(.caption)
                .foregroundStyle(.secondary)
        }
        .padding(16)
        .background(Color.orange.opacity(0.08), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: 16, style: .continuous)
                .stroke(Color.orange.opacity(0.35), lineWidth: 1)
        )
    }
}

struct GuidedBringupSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    private var resultObject: [String: JSONValue]? {
        vm.guidedBringupResponse?.result?.objectValue
    }

    private var stageObjects: [[String: JSONValue]] {
        resultObject?["stages"]?.arrayValue?.compactMap(\.objectValue) ?? []
    }

    private var overallOk: Bool? {
        resultObject?["ok"]?.boolValue
    }

    private var failedStage: String? {
        resultObject?["failed_stage"]?.stringValue
    }

    private func stageBadgeColor(for stage: [String: JSONValue]) -> Color {
        if stage["ok"]?.boolValue == true {
            return (stage["skipped"]?.boolValue == true) ? .gray : .green
        }
        return .red
    }

    private func stageStatusText(for stage: [String: JSONValue]) -> String {
        if stage["ok"]?.boolValue == true {
            return (stage["skipped"]?.boolValue == true) ? "skipped" : "passed"
        }
        return "failed"
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Text("Guided Bring-Up")
                    .font(.title2.bold())
                Spacer()
                Toggle("Persist detected direction", isOn: $vm.guidedBringupPersistDirection)
                    .toggleStyle(.switch)
                Button("Run Guided Bring-Up") {
                    Task { await vm.runGuidedBringup() }
                }
                .buttonStyle(.borderedProminent)
                .disabled(vm.isBusy || vm.capabilities?.motion_active == true)
            }

            Text("Runs a bounded staged bring-up on the currently selected board and axis: clear/idle, motor calibration if needed, encoder preflight using current encoder config, guarded startup contract, auto direction contract, and a small position sign probe. This is not blind profile autotune.")
                .foregroundStyle(.secondary)

            if let overallOk {
                HStack(spacing: 12) {
                    StatusBadge(
                        title: "Bring-Up Result",
                        value: overallOk ? "passed" : "failed",
                        color: overallOk ? .green : .red
                    )
                    if let failedStage, !failedStage.isEmpty {
                        StatusBadge(
                            title: "Failed Stage",
                            value: failedStage,
                            color: .red
                        )
                    }
                }
            }

            if stageObjects.isEmpty {
                Text("No guided bring-up run yet.")
                    .foregroundStyle(.secondary)
            } else {
                ForEach(Array(stageObjects.enumerated()), id: \.offset) { _, stage in
                    VStack(alignment: .leading, spacing: 8) {
                        HStack {
                            Text(stage["name"]?.stringValue ?? "stage")
                                .font(.headline)
                            Spacer()
                            Text(stageStatusText(for: stage))
                                .font(.subheadline.weight(.semibold))
                                .padding(.horizontal, 10)
                                .padding(.vertical, 4)
                                .background(stageBadgeColor(for: stage).opacity(0.18), in: Capsule())
                        }
                        if let message = stage["message"]?.stringValue, !message.isEmpty {
                            Text(message)
                        }
                        if let details = stage["details"]?.objectValue, !details.isEmpty {
                            ForEach(details.keys.sorted(), id: \.self) { key in
                                if let value = details[key] {
                                    HStack(alignment: .firstTextBaseline) {
                                        Text(key)
                                            .font(.system(.caption, design: .monospaced))
                                            .foregroundStyle(.secondary)
                                        Spacer()
                                        Text(value.description)
                                            .font(.system(.caption, design: .monospaced))
                                            .multilineTextAlignment(.trailing)
                                    }
                                }
                            }
                        }
                        if let logs = stage["logs"]?.stringValue, !logs.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                            ScrollView {
                                Text(logs)
                                    .font(.system(.caption, design: .monospaced))
                                    .textSelection(.enabled)
                                    .frame(maxWidth: .infinity, alignment: .leading)
                            }
                            .frame(minHeight: 70, maxHeight: 140)
                            .padding(8)
                            .background(Color(nsColor: .textBackgroundColor), in: RoundedRectangle(cornerRadius: 8, style: .continuous))
                        }
                    }
                    .padding(12)
                    .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
                }
            }
        }
        .padding(16)
        .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
    }
}

private struct SlewRuntimeTuningView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @State private var isExpanded = false

    private var editor: ProfileEditorFormState? { vm.selectedProfileEditorLoaded ? vm.profileEditor : nil }

    private var baseCommandVel: Double? {
        guard let editor else { return nil }
        return Double(editor.commandVelTurnsS)
    }

    private var speedScale: Double? {
        Double(vm.moveForm.runtimeSpeedScale.trimmingCharacters(in: .whitespacesAndNewlines))
    }

    private var effectiveCommandVel: Double? {
        guard let baseCommandVel, let speedScale, speedScale > 0 else { return nil }
        return baseCommandVel * speedScale
    }

    private var gearRatio: Double? {
        let parsed = Double(vm.moveForm.gearRatio.trimmingCharacters(in: .whitespacesAndNewlines))
        guard let parsed, parsed > 0 else { return nil }
        return parsed
    }

    private var estimatedOutputDegS: Double? {
        guard vm.moveForm.angleSpace == "gearbox_output",
              let effectiveCommandVel,
              let gearRatio
        else { return nil }
        return (effectiveCommandVel * 360.0) / gearRatio
    }

    private var estimatedMotorDegS: Double? {
        guard let effectiveCommandVel else { return nil }
        return effectiveCommandVel * 360.0
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            DisclosureGroup(isExpanded: $isExpanded) {
                VStack(alignment: .leading, spacing: 10) {
                    Text("Runtime only. This scales the travel speed for the selected experimental travel-shaped profile without editing or saving the profile.")
                        .font(.caption)
                        .foregroundStyle(.secondary)

                    HStack(alignment: .top, spacing: 12) {
                        LabeledInputField(title: "Speed scale x", text: $vm.moveForm.runtimeSpeedScale)
                        VStack(alignment: .leading, spacing: 6) {
                            Text("Effective speed")
                                .font(.caption.weight(.medium))
                                .foregroundStyle(.secondary)
                            if let baseCommandVel {
                                Text(String(format: "Base: %.3f motor turns/s", baseCommandVel))
                                    .font(.system(.body, design: .monospaced))
                            }
                            if let effectiveCommandVel {
                                Text(String(format: "Now: %.3f motor turns/s", effectiveCommandVel))
                                    .font(.system(.body, design: .monospaced))
                                if let estimatedOutputDegS {
                                    Text(String(format: "%.1f output deg/s", estimatedOutputDegS))
                                        .font(.system(.body, design: .monospaced))
                                } else if let estimatedMotorDegS {
                                    Text(String(format: "%.1f motor deg/s", estimatedMotorDegS))
                                        .font(.system(.body, design: .monospaced))
                                }
                            } else {
                                Text("Enter a positive scale value.")
                                    .foregroundStyle(.secondary)
                            }
                        }
                        .frame(maxWidth: .infinity, alignment: .leading)
                    }
                }
                .padding(.top, 8)
            } label: {
                HStack {
                    VStack(alignment: .leading, spacing: 2) {
                        Text("Runtime Speed Tweak")
                            .font(.headline)
                        if let effectiveCommandVel {
                            if let estimatedOutputDegS {
                                Text(String(format: "Scale %.2fx • %.3f t/s • %.1f output deg/s", speedScale ?? 1.0, effectiveCommandVel, estimatedOutputDegS))
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                            } else {
                                Text(String(format: "Scale %.2fx • %.3f t/s", speedScale ?? 1.0, effectiveCommandVel))
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                            }
                        } else {
                            Text("Collapsed by default. Open only when testing travel-speed changes.")
                                .font(.caption)
                                .foregroundStyle(.secondary)
                        }
                    }
                    Spacer()
                    Button("Reset") {
                        vm.moveForm.runtimeSpeedScale = "1.0"
                    }
                }
            }
        }
        .padding(14)
        .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
    }
}

struct ProfileEditorSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @State private var isExpanded = false

    private var moveMode: String { vm.profileEditor.moveMode.trimmingCharacters(in: .whitespacesAndNewlines).lowercased() }
    private var isTrapProfile: Bool { moveMode == "trap_strict" || moveMode.isEmpty }
    private var isDirectProfile: Bool {
        moveMode == "mks_directional_direct"
            || moveMode == "mks_directional_slew_direct"
            || moveMode == "mks_directional_velocity_travel_direct"
            || moveMode == "mks_velocity_point_to_point_direct"
    }
    private var isShapedTravelProfile: Bool {
        moveMode == "mks_directional_slew_direct"
            || moveMode == "mks_directional_velocity_travel_direct"
            || moveMode == "mks_velocity_point_to_point_direct"
    }
    private var isReadOnlyBuiltIn: Bool { vm.profileEditor.isBuiltInReadOnly }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            DisclosureGroup(isExpanded: $isExpanded) {
                VStack(alignment: .leading, spacing: 12) {
                    if isReadOnlyBuiltIn {
                        HStack(alignment: .top, spacing: 12) {
                            VStack(alignment: .leading, spacing: 4) {
                                Text("Built-in profile: read-only")
                                    .font(.subheadline.weight(.semibold))
                                Text("Fork it first if you want to edit or save a variant. Runtime speed tests belong in the move panel above, not in the saved profile.")
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                            }
                            Spacer()
                            Button("Fork to Editable Copy") {
                                vm.forkLoadedProfileEditor()
                            }
                            .buttonStyle(.borderedProminent)
                        }
                    } else {
                        Text("The editor is preloaded from the selected dropdown profile. Save with the same name to overwrite it, or change the name to fork a new profile.")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }

                    Group {
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
                            Text(isDirectProfile ? "Final Settle Gains / Limits" : "Motion Gains / Limits")
                                .font(.headline)
                            if isDirectProfile {
                                Text("`Run current A` is the motion current ceiling. These are the canonical saved gains and the final-settle gains. They are not the travel-shaping overrides used during shaped direct moves.")
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                            }
                            HStack(alignment: .top, spacing: 12) {
                                LabeledInputField(title: "Run current A", text: $vm.profileEditor.currentLim)
                                LabeledInputField(title: "Pos gain", text: $vm.profileEditor.posGain)
                                LabeledInputField(title: "Vel gain", text: $vm.profileEditor.velGain)
                                LabeledInputField(title: "Vel I gain", text: $vm.profileEditor.velIGain)
                            }
                            HStack(alignment: .top, spacing: 12) {
                                LabeledInputField(title: isDirectProfile ? "Trap vel (unused)" : "Trap vel", text: $vm.profileEditor.trapVel)
                                LabeledInputField(title: isDirectProfile ? "Trap acc (unused)" : "Trap acc", text: $vm.profileEditor.trapAcc)
                                LabeledInputField(title: isDirectProfile ? "Trap dec (unused)" : "Trap dec", text: $vm.profileEditor.trapDec)
                            }
                            if isDirectProfile {
                                Text("Direct MKS profiles do not use trap vel/acc/dec during motion. Travel aggressiveness comes from the direct move-law fields below: `Cmd vel t/s`, `Cmd dt s`, `Handoff turns`, and the travel-only gains/limits.")
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                            }
                            HStack(alignment: .top, spacing: 12) {
                                LabeledInputField(title: "Vel limit", text: $vm.profileEditor.velLimit)
                                LabeledInputField(title: "Vel limit tol", text: $vm.profileEditor.velLimitTolerance)
                                LabeledInputField(title: "Stiction kick Nm", text: $vm.profileEditor.stictionKickNm)
                            }
                        }

                        if isDirectProfile {
                            VStack(alignment: .leading, spacing: 8) {
                                Text("Direct Move Law")
                                    .font(.headline)
                                Text("These fields control the direct-position move helper itself. They are the values that were previously only visible in the new summary card.")
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                                HStack(alignment: .top, spacing: 12) {
                                    LabeledInputField(title: "Pre hold s", text: $vm.profileEditor.preHoldS)
                                    LabeledInputField(title: "Final hold s", text: $vm.profileEditor.finalHoldS)
                                    LabeledInputField(title: "Abort abs turns", text: $vm.profileEditor.abortAbsTurns)
                                }
                            }

                            VStack(alignment: .leading, spacing: 8) {
                                Text("Startup Calibration")
                                    .font(.headline)
                                Text("These values control startup recovery for direct MKS profiles. `Motor cal A` and `Encoder cal A` affect calibration only; they do not set move torque. Use `Run current A` for runtime motion current.")
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                                HStack(alignment: .top, spacing: 12) {
                                    LabeledInputField(title: "Pole pairs", text: $vm.profileEditor.polePairs)
                                    LabeledInputField(title: "Motor cal A", text: $vm.profileEditor.calibrationCurrent)
                                    LabeledInputField(title: "Encoder cal A", text: $vm.profileEditor.encoderOffsetCalibrationCurrent)
                                }
                            }
                        }

                        if isShapedTravelProfile {
                            VStack(alignment: .leading, spacing: 8) {
                                Text("Travel Overrides")
                                    .font(.headline)
                                Text(
                                    moveMode == "mks_velocity_point_to_point_direct"
                                    ? "These apply only during the velocity-led travel phase. `Handoff turns` is the distance from target where velocity mode yields to final position capture."
                                    : "These apply only during the travel phase. Final settle reverts to the base gains above."
                                )
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                                HStack(alignment: .top, spacing: 12) {
                                    LabeledInputField(title: "Cmd vel t/s", text: $vm.profileEditor.commandVelTurnsS)
                                    LabeledInputField(title: "Handoff turns", text: $vm.profileEditor.handoffWindowTurns)
                                    LabeledInputField(title: "Cmd dt s", text: $vm.profileEditor.commandDt)
                                }
                                HStack(alignment: .top, spacing: 12) {
                                    LabeledInputField(title: "Travel pos gain", text: $vm.profileEditor.travelPosGain)
                                    LabeledInputField(title: "Travel vel gain", text: $vm.profileEditor.travelVelGain)
                                    LabeledInputField(title: "Travel vel I", text: $vm.profileEditor.travelVelIGain)
                                    LabeledInputField(title: "Travel vel limit", text: $vm.profileEditor.travelVelLimit)
                                }
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
                }
                .disabled(isReadOnlyBuiltIn)
                .opacity(isReadOnlyBuiltIn ? 0.65 : 1.0)
                .padding(.top, 8)
            } label: {
                HStack {
                    VStack(alignment: .leading, spacing: 2) {
                        Text("Edit / Fork Profile")
                            .font(.title3.bold())
                        Text(isReadOnlyBuiltIn ? "Built-in profiles are read-only until forked." : "Collapsed by default. Open only when editing or forking a profile.")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                    Spacer()
                    Button("Reload Selected") {
                        Task { await vm.loadProfileEditor() }
                    }
                    .disabled(vm.moveForm.profileName.isEmpty)
                    Button(isReadOnlyBuiltIn ? "Fork Built-in" : "Save Profile") {
                        if isReadOnlyBuiltIn {
                            vm.forkLoadedProfileEditor()
                            isExpanded = true
                        } else {
                            Task { await vm.saveProfileEditor() }
                        }
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(vm.profileEditor.name.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty)
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
    private var continuousMoveDisabledReason: String? {
        if vm.isBusy {
            return "Another action is currently running."
        }
        guard let capabilities else {
            return "Board status is not loaded yet."
        }
        if capabilities.motion_active == true {
            return "A background move is still active."
        }
        if capabilities.can_move_continuous == true {
            return nil
        }
        if capabilities.startup_ready != true {
            return "Axis is not startup-ready."
        }
        if capabilities.has_latched_errors == true {
            return "Latched axis/controller errors are present."
        }
        return "Continuous move is currently unavailable for this board state."
    }

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

            SelectedProfileSummaryView(vm: vm)

            if vm.selectedProfileSupportsRuntimeSpeedTweak {
                SlewRuntimeTuningView(vm: vm)
            }

            ProfileEditorSectionView(vm: vm)

            Button("Run Continuous Move") {
                Task { await vm.moveContinuous() }
            }
            .buttonStyle(.borderedProminent)
            .disabled(vm.isBusy || capabilities?.can_move_continuous != true || capabilities?.motion_active == true)

            if let reason = continuousMoveDisabledReason {
                Text(reason)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }

            DirectControlCardView(vm: vm, live: live)
            VelocityFloorAssistCardView(vm: vm)
            DirectRunQualityCardView(vm: vm, live: live)

            MoveDiagnosticsCardView(vm: vm)
        }
        .padding(16)
        .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
    }
}

private struct DirectControlCardView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @ObservedObject var live: LiveMonitorModel

    private var capabilities: BackendCapabilities? { live.capabilities ?? vm.response?.capabilities }

    private var disabledReason: String? {
        if capabilities == nil {
            return "No motor status yet. Click Refresh Status first."
        }
        if vm.isBusy {
            return "Another action is currently running."
        }
        if capabilities?.motion_active == true {
            return "A background move is still active."
        }
        if capabilities?.startup_ready != true {
            return "Axis is not startup-ready."
        }
        if vm.directControlForm.mode == "position",
           vm.directControlForm.releaseAfterPosition,
           vm.directControlForm.timeoutSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            return "Timeout is required when releasing after a position move."
        }
        if vm.directControlForm.mode == "velocity",
           vm.directControlForm.releaseAfterVelocity,
           vm.directControlForm.durationSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            return "Duration is required when releasing after a velocity spin."
        }
        return nil
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                VStack(alignment: .leading, spacing: 4) {
                    Text("Direct Control")
                        .font(.headline)
                    Text("Raw motor-space commands that bypass the profile travel logic. Use these for proof-of-life and bounded manual tests.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
                Spacer()
                Picker("Mode", selection: $vm.directControlForm.mode) {
                    Text("Position").tag("position")
                    Text("Velocity").tag("velocity")
                }
                .pickerStyle(.segmented)
                .frame(width: 210)
            }

            if vm.directControlForm.mode == "position" {
                VStack(alignment: .leading, spacing: 10) {
                    HStack(alignment: .top, spacing: 12) {
                        LabeledInputField(title: "Motor turns", text: $vm.directControlForm.turns)
                        LabeledInputField(title: "Timeout (s)", text: $vm.directControlForm.timeoutSeconds)
                    }
                    Toggle("Relative to current position", isOn: $vm.directControlForm.relativeTurns)
                    HStack(alignment: .top, spacing: 12) {
                        LabeledInputField(title: "Target tol turns", text: $vm.directControlForm.targetToleranceTurns)
                        LabeledInputField(title: "Target vel tol", text: $vm.directControlForm.targetVelToleranceTurnsS)
                    }
                    Toggle("Release to IDLE after timeout-waited move", isOn: $vm.directControlForm.releaseAfterPosition)
                    Text("`1.0` means one motor turn. With a 25:1 gearbox, `25.0` motor turns equals one full output turn.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    Button("Send Position Command") {
                        Task { await vm.commandDirectPosition() }
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(disabledReason != nil)
                }
            } else {
                VStack(alignment: .leading, spacing: 10) {
                    HStack(alignment: .top, spacing: 12) {
                        LabeledInputField(title: "Motor turns/s", text: $vm.directControlForm.turnsPerSecond)
                        LabeledInputField(title: "Duration (s, optional)", text: $vm.directControlForm.durationSeconds)
                    }
                    Toggle("Release to IDLE after timed spin", isOn: $vm.directControlForm.releaseAfterVelocity)
                    Text("Leave duration empty only if you intend to stop the axis manually with `Idle (1)` or another state command.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    HStack {
                        Button("Send Velocity Command") {
                            Task { await vm.commandDirectVelocity() }
                        }
                        .buttonStyle(.borderedProminent)
                        .disabled(disabledReason != nil)
                        Button("Send Assisted Velocity") {
                            Task { await vm.commandAssistedVelocity() }
                        }
                        .buttonStyle(.bordered)
                        .disabled(vm.assistedVelocityDisabledReason != nil)
                    }
                    Text("Assisted velocity is the first output-aware low-speed mode. It kicks above the detected floor, waits for real output breakaway, then returns to your requested speed.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }

            if let disabledReason {
                Text(disabledReason)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            Text("Use the Run Quality card below to judge real output motion, direction, lag growth, and stop quality.")
                .font(.caption)
                .foregroundStyle(.secondary)
        }
        .padding(12)
        .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
    }
}

private struct VelocityFloorAssistCardView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    private var sweep: VelocitySweepSummary? { vm.latestVelocitySweep }

    private func tint(for tier: DirectVelocityHint.Tier) -> Color {
        switch tier {
        case .usable:
            return .green
        case .borderline:
            return .orange
        case .belowFloor:
            return .red
        case .unknown:
            return .gray
        }
    }

    private func tint(for tier: VelocitySweepPoint.Tier) -> Color {
        switch tier {
        case .usable:
            return .green
        case .borderline:
            return .orange
        case .belowFloor, .faulted:
            return .red
        }
    }

    private func statRow(_ title: String, _ value: String) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(title)
                .font(.caption2)
                .foregroundStyle(.secondary)
            Text(value)
                .font(.system(.subheadline, design: .monospaced))
                .fontWeight(.medium)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    private func compact(_ value: Double?, suffix: String) -> String {
        guard let value else { return "not found" }
        return String(format: "%.2f %@", value, suffix)
    }

    private func compactPercent(_ value: Double?) -> String {
        guard let value else { return "unknown" }
        return String(format: "%.0f%%", value * 100.0)
    }

    private func compactTurns(_ value: Double?, suffix: String = "t") -> String {
        guard let value else { return "unknown" }
        return String(format: "%+.4f %@", value, suffix)
    }

    private var sweepShowsMotorWithoutOutputBreakaway: Bool {
        guard let sweep else { return false }
        return sweep.points.contains { point in
            let motorFollow = point.quality.motorFollowFraction ?? 0.0
            let transmission = point.quality.transmissionFollowFraction ?? 0.0
            let peakMotor = abs(point.quality.furthestMotorDeltaTurns ?? 0.0)
            let peakOutput = abs(point.quality.furthestOutputDeltaTurns ?? 0.0)
            return motorFollow >= 0.25 && peakMotor >= 0.10 && transmission < 0.10 && peakOutput < 0.0015
        }
    }

    var body: some View {
        if vm.directControlForm.mode == "velocity" || sweep != nil {
            VStack(alignment: .leading, spacing: 12) {
                HStack {
                    VStack(alignment: .leading, spacing: 4) {
                        Text("Low-Speed Floor")
                            .font(.headline)
                        Text("Use output-angle evidence to find the breakaway floor, then use assisted velocity only when the plain command is below or near that floor.")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                    Spacer()
                    Text(vm.currentVelocityHint.headline)
                        .font(.caption.weight(.semibold))
                        .padding(.horizontal, 10)
                        .padding(.vertical, 6)
                        .background(tint(for: vm.currentVelocityHint.tier).opacity(0.15), in: Capsule())
                }

                Text(vm.currentVelocityHint.detail)
                    .font(.caption)
                    .foregroundStyle(.secondary)

                if sweepShowsMotorWithoutOutputBreakaway {
                    Text("Sweep evidence says the motor is moving, but no real output breakaway has been detected yet. Treat this as a transmission/output-sensing problem, not a successful low-speed floor.")
                        .font(.caption)
                        .foregroundStyle(.orange)
                }

                HStack(alignment: .top, spacing: 16) {
                    statRow("Breakaway floor", compact(vm.detectedBreakawayFloorTurnsPerSecond, suffix: "t/s"))
                    statRow("Usable floor", compact(vm.detectedUsableFloorTurnsPerSecond, suffix: "t/s"))
                    statRow(
                        "Assist floor",
                        vm.assistFloorTurnsPerSecond.map { String(format: "%.2f t/s (%@)", $0, vm.assistFloorSourceLabel ?? "derived") } ?? "run sweep first"
                    )
                }

                VStack(alignment: .leading, spacing: 10) {
                    Text("Automatic Sweep")
                        .font(.subheadline.weight(.semibold))
                    HStack(alignment: .top, spacing: 12) {
                        LabeledInputField(title: "Start t/s", text: $vm.directControlForm.sweepStartTurnsPerSecond)
                        LabeledInputField(title: "Stop t/s", text: $vm.directControlForm.sweepStopTurnsPerSecond)
                        LabeledInputField(title: "Step t/s", text: $vm.directControlForm.sweepStepTurnsPerSecond)
                        LabeledInputField(title: "Duration (s)", text: $vm.directControlForm.sweepDurationSeconds)
                    }
                    Button("Run Breakaway Sweep") {
                        Task { await vm.runVelocityBreakawaySweep() }
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(vm.velocitySweepDisabledReason != nil)
                    if let reason = vm.velocitySweepDisabledReason {
                        Text(reason)
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }

                VStack(alignment: .leading, spacing: 10) {
                    Text("Output-Aware Assist")
                        .font(.subheadline.weight(.semibold))
                    HStack(alignment: .top, spacing: 12) {
                        LabeledInputField(title: "Manual floor t/s", text: $vm.directControlForm.assistManualFloorTurnsPerSecond)
                        LabeledInputField(title: "Kick max (s)", text: $vm.directControlForm.assistKickMaxDurationSeconds)
                        LabeledInputField(title: "Breakaway out t", text: $vm.directControlForm.assistBreakawayOutputTurns)
                    }
                    if let reason = vm.assistedVelocityDisabledReason {
                        Text(reason)
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    } else {
                        Text("Assist will use \(vm.assistFloorSourceLabel ?? "derived floor") at \(compact(vm.assistFloorTurnsPerSecond, suffix: "t/s")).")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }

                if let sweep, !sweep.points.isEmpty {
                    Divider()
                    Text("Last Sweep")
                        .font(.subheadline.weight(.semibold))
                    VStack(alignment: .leading, spacing: 8) {
                        ForEach(Array(sweep.points.enumerated()), id: \.offset) { _, point in
                            HStack(alignment: .top, spacing: 12) {
                                Text(String(format: "%.2f t/s", point.commandTurnsPerSecond))
                                    .font(.system(.body, design: .monospaced))
                                    .frame(width: 88, alignment: .leading)
                                Text(point.quality.headline)
                                    .font(.caption.weight(.semibold))
                                    .padding(.horizontal, 8)
                                    .padding(.vertical, 4)
                                    .background(tint(for: point.tier).opacity(0.15), in: Capsule())
                                Spacer()
                                VStack(alignment: .trailing, spacing: 2) {
                                    Text("motor \(compactPercent(point.quality.motorFollowFraction)) • trans \(compactPercent(point.quality.transmissionFollowFraction))")
                                        .font(.system(.caption, design: .monospaced))
                                        .foregroundStyle(.secondary)
                                    Text("peak m \(compactTurns(point.quality.furthestMotorDeltaTurns)) • peak out \(compactTurns(point.quality.furthestOutputDeltaTurns, suffix: "out t"))")
                                        .font(.system(.caption, design: .monospaced))
                                        .foregroundStyle(.secondary)
                                }
                            }
                        }
                    }
                    if let stoppedReason = sweep.stoppedReason, !stoppedReason.isEmpty {
                        Text("Sweep stopped early: \(stoppedReason)")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }
            }
            .padding(12)
            .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
        }
    }
}

private struct DirectRunQualityCardView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @ObservedObject var live: LiveMonitorModel

    private var baseline: DirectRunBaseline? { vm.activeDirectRunBaseline }
    private var quality: DirectRunQuality? { vm.latestDirectRunQuality }
    private var currentSnapshot: BackendSnapshot? { live.snapshot ?? vm.response?.snapshot }
    private var currentSensor: BackendOutputSensor? { live.outputSensor ?? vm.response?.output_sensor }

    private func verdictColor(_ verdict: DirectRunQuality.Verdict) -> Color {
        switch verdict {
        case .good:
            return .green
        case .partial, .signInverted:
            return .orange
        case .stalled, .sensorMismatch, .wrongDirection, .faulted:
            return .red
        case .informational:
            return .gray
        }
    }

    private func statRow(_ title: String, _ value: String) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(title)
                .font(.caption2)
                .foregroundStyle(.secondary)
            Text(value)
                .font(.system(.subheadline, design: .monospaced))
                .fontWeight(.medium)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    private func formattedTurns(_ value: Double?, suffix: String = "t") -> String {
        guard let value else { return "unknown" }
        return String(format: "%+.4f %@", value, suffix)
    }

    private func formattedFraction(_ value: Double?) -> String {
        guard let value else { return "unknown" }
        return String(format: "%.0f%%", value * 100.0)
    }

    private func currentMotorDelta(from baseline: DirectRunBaseline) -> Double? {
        guard let start = baseline.startMotorTurns, let end = currentSnapshot?.pos_est else { return nil }
        return end - start
    }

    private func currentOutputDelta(from baseline: DirectRunBaseline) -> Double? {
        guard let start = baseline.startOutputTurns, let end = currentSensor?.output_turns else { return nil }
        return end - start
    }

    private func currentLagDelta(from baseline: DirectRunBaseline) -> Double? {
        guard let start = baseline.startLagOutputTurns, let end = currentSensor?.compliance_lag_output_turns else { return nil }
        return end - start
    }

    var body: some View {
        if let baseline {
            VStack(alignment: .leading, spacing: 10) {
                HStack {
                    Text("Run Quality")
                        .font(.headline)
                    Spacer()
                    ProgressView()
                        .controlSize(.small)
                }

                Text("Run in progress. Watch the real output delta and lag delta; the scored verdict appears when the run completes.")
                    .font(.caption)
                    .foregroundStyle(.secondary)

                HStack(alignment: .top, spacing: 16) {
                    statRow("Mode", baseline.mode)
                    statRow("Motor Δ", formattedTurns(currentMotorDelta(from: baseline)))
                    statRow("Output Δ", formattedTurns(currentOutputDelta(from: baseline), suffix: "out t"))
                    statRow("Lag Δ", formattedTurns(currentLagDelta(from: baseline), suffix: "out t"))
                    statRow("Output vel", formattedTurns(currentSensor?.output_vel_turns_s, suffix: "out t/s"))
                }
            }
            .padding(12)
            .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
        } else if let quality {
            VStack(alignment: .leading, spacing: 10) {
                HStack {
                    Text("Run Quality")
                        .font(.headline)
                    Spacer()
                    Text(quality.headline)
                        .font(.caption.weight(.semibold))
                        .padding(.horizontal, 10)
                        .padding(.vertical, 6)
                        .background(verdictColor(quality.verdict).opacity(0.15), in: Capsule())
                }

                Text(quality.explanation)
                    .font(.caption)
                    .foregroundStyle(.secondary)

                HStack(alignment: .top, spacing: 16) {
                    statRow("Mode", quality.mode)
                    statRow(
                        "Direction",
                        {
                            if quality.verdict == .signInverted {
                                return "sign inverted"
                            }
                            return quality.directionCorrect.map { $0 ? "correct" : "wrong" } ?? "unclear"
                        }()
                    )
                    statRow("Motor follow", formattedFraction(quality.motorFollowFraction))
                    statRow("Cmd->out", formattedFraction(quality.outputFollowFraction))
                    statRow(
                        "Stop",
                        quality.finalOutputVelTurnsS.map { abs($0) <= 0.01 ? "settled" : "still moving" } ?? "unknown"
                    )
                }

                HStack(alignment: .top, spacing: 16) {
                    statRow(
                        "Motor exp / act",
                        "\(formattedTurns(quality.expectedMotorDeltaTurns)) / \(formattedTurns(quality.actualMotorDeltaTurns))"
                    )
                    statRow("Peak motor Δ", formattedTurns(quality.furthestMotorDeltaTurns))
                    statRow(
                        "Output exp / act",
                        "\(formattedTurns(quality.expectedOutputDeltaTurns, suffix: "out t")) / \(formattedTurns(quality.actualOutputDeltaTurns, suffix: "out t"))"
                    )
                    statRow(
                        "Output from motor",
                        formattedTurns(quality.expectedOutputFromActualMotorTurns, suffix: "out t")
                    )
                    statRow("Peak output Δ", formattedTurns(quality.furthestOutputDeltaTurns, suffix: "out t"))
                    statRow("Transmission", formattedFraction(quality.transmissionFollowFraction))
                    statRow("Lag Δ", formattedTurns(quality.lagDeltaOutputTurns, suffix: "out t"))
                    statRow("Peak motor vel", formattedTurns(quality.peakMotorVelTurnsS, suffix: "t/s"))
                    statRow("Peak output vel", formattedTurns(quality.peakOutputVelTurnsS, suffix: "out t/s"))
                }

                if let reached = quality.motorReachedTarget {
                    Text("Motor-side target: \(reached ? "reached" : "not reached")")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }

                if quality.assistUsed {
                    HStack(alignment: .top, spacing: 16) {
                        statRow("Assist floor", formattedTurns(quality.assistFloorTurnsPerSecond, suffix: "t/s"))
                        statRow("Kick", formattedTurns(quality.assistKickTurnsPerSecond, suffix: "t/s"))
                        statRow("Kick duration", quality.assistKickDurationS.map { String(format: "%.3f s", $0) } ?? "unknown")
                        statRow(
                            "Breakaway",
                            quality.assistBreakawayDetected.map { $0 ? "detected" : "not detected" } ?? "unknown"
                        )
                    }
                    if let detectedAt = quality.assistBreakawayDetectedAtS {
                        Text(String(format: "Assist breakaway detected at %.3f s.", detectedAt))
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }

                if let errorSummary = quality.errorSummary, !errorSummary.isEmpty {
                    Text("Errors: \(errorSummary)")
                        .font(.caption)
                        .foregroundStyle(.red)
                }
            }
            .padding(12)
            .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
        }
    }
}

private struct SelectedProfileSummaryView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @State private var showAdvanced = false
    @State private var showLimitations = false

    private var detail: BackendProfileDetail? { vm.selectedProfileDetail }
    private var editor: ProfileEditorFormState? { vm.selectedProfileEditorLoaded ? vm.profileEditor : nil }

    private var hasContent: Bool { detail != nil || editor != nil }

    private func metricRow(_ title: String, _ value: String) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(title)
                .font(.caption)
                .foregroundStyle(.secondary)
            Text(value)
                .font(.system(.body, design: .monospaced))
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(10)
        .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 10, style: .continuous))
    }

    private func boolBadge(_ title: String, _ enabled: Bool, tint: Color? = nil) -> some View {
        Text("\(title): \(enabled ? "yes" : "no")")
            .font(.caption.weight(.semibold))
            .padding(.horizontal, 10)
            .padding(.vertical, 6)
            .background((tint ?? (enabled ? .green : .gray)).opacity(0.15), in: Capsule())
    }

    private func labelBadge(_ title: String, tint: Color) -> some View {
        Text(title)
            .font(.caption.weight(.semibold))
            .padding(.horizontal, 10)
            .padding(.vertical, 6)
            .background(tint.opacity(0.15), in: Capsule())
    }

    private func compactStat(_ title: String, _ value: String) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(title)
                .font(.caption2)
                .foregroundStyle(.secondary)
            Text(value)
                .font(.system(.subheadline, design: .monospaced))
                .fontWeight(.medium)
        }
    }

    private var isCoarseOnlyProfile: Bool {
        let notes = (editor?.notes ?? detail?.notes ?? "").lowercased()
        let limitations = ((editor?.limitationsText ?? "") + "\n" + ((detail?.limitations)?.joined(separator: "\n") ?? "")).lowercased()
        return notes.contains("coarse-motion")
            || notes.contains("coarse mounted motion")
            || limitations.contains("coarse mounted motion")
            || limitations.contains("not precision arm control")
            || limitations.contains("not a precision profile")
    }

    private var isBareOnlyProfile: Bool {
        guard let editor else { return false }
        let profileName = editor.name.trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
        let candidatePreset = editor.candidatePreset.trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
        let notes = (editor.notes + "\n" + (detail?.notes ?? "")).lowercased()
        return profileName.contains("bare")
            || candidatePreset.contains("bare")
            || notes.contains("bare-motor")
    }

    private var startupWarning: String? {
        guard let editor else { return nil }
        let moveMode = editor.moveMode.trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
        guard moveMode == "mks_directional_direct"
                || moveMode == "mks_directional_slew_direct"
                || moveMode == "mks_directional_velocity_travel_direct"
        else { return nil }

        let profileName = editor.name.trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
        let candidatePreset = editor.candidatePreset.trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
        if profileName.contains("bare") || candidatePreset.contains("bare") {
            return "This is a bare-motor MKS profile. Do not use it while the gearbox is mounted. Use a mounted MKS profile family instead."
        }

        if editor.enableOverspeedError {
            return "Overspeed error is enabled on a direct MKS profile. That is a risky startup family for this setup."
        }
        if let velTol = Double(editor.velLimitTolerance), velTol < 4.0 {
            return "Vel limit tolerance is below 4.0. That does not match the safer direct MKS startup family."
        }
        if editor.name == "mks_mounted_direct_preload_coarse_v1_exp" {
            let polePairs = Int(editor.polePairs.trimmingCharacters(in: .whitespacesAndNewlines))
            let motorCal = Double(editor.calibrationCurrent.trimmingCharacters(in: .whitespacesAndNewlines))
            let encoderCal = Double(editor.encoderOffsetCalibrationCurrent.trimmingCharacters(in: .whitespacesAndNewlines))
            if polePairs != 7 || motorCal != 2.0 || encoderCal != 8.0 {
                return "This coarse mounted MKS profile is not on its proven startup family. Expected Pole pairs 7, Motor cal 2.0 A, Encoder cal 8.0 A."
            }
        }
        return nil
    }

    var body: some View {
        if hasContent {
            VStack(alignment: .leading, spacing: 10) {
                HStack {
                    Text("Selected Profile")
                        .font(.headline)
                    Spacer()
                    if let editor {
                        Text(editor.moveMode.replacingOccurrences(of: "_", with: " "))
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                }

                if let notes = detail?.notes, !notes.isEmpty {
                    Text(notes)
                        .lineLimit(3)
                }

                if let editor {
                    if let startupWarning {
                        HStack(alignment: .top, spacing: 8) {
                            Text("Startup Warning")
                                .font(.caption.weight(.semibold))
                                .padding(.horizontal, 10)
                                .padding(.vertical, 6)
                                .background(Color.red.opacity(0.15), in: Capsule())
                            Text(startupWarning)
                                .font(.caption)
                                .foregroundStyle(.secondary)
                            Spacer()
                        }
                    }

                    HStack(spacing: 8) {
                        boolBadge("Experimental", editor.experimental, tint: .orange)
                        boolBadge("Foundation Validated", editor.foundationValidated, tint: .blue)
                        boolBadge("Live Follow", editor.liveFollowSupported, tint: .purple)
                        if isBareOnlyProfile {
                            labelBadge("Bare Only", tint: .orange)
                        }
                        if isCoarseOnlyProfile {
                            labelBadge("Coarse Only", tint: .red)
                        }
                    }

                    HStack(alignment: .top, spacing: 16) {
                        compactStat("Profile", editor.name)
                        if !editor.candidatePreset.isEmpty {
                            compactStat("Candidate", editor.candidatePreset)
                        }
                        compactStat("Pos / Vel", "\(editor.posGain) / \(editor.velGain)")
                        compactStat("Vel limit", editor.velLimit)
                        if (editor.moveMode == "mks_directional_slew_direct" || editor.moveMode == "mks_directional_velocity_travel_direct"), !editor.commandVelTurnsS.isEmpty {
                            compactStat("Cmd vel t/s", editor.commandVelTurnsS)
                        } else {
                            compactStat("Timeout s", editor.timeoutS)
                        }
                    }

                    DisclosureGroup(isExpanded: $showAdvanced) {
                        VStack(alignment: .leading, spacing: 10) {
                            LazyVGrid(columns: [GridItem(.adaptive(minimum: 110), spacing: 8)], spacing: 8) {
                                metricRow("Move mode", editor.moveMode)
                                metricRow("Load mode", editor.loadMode)
                                metricRow("Source", editor.source)
                                metricRow("Run current A", editor.currentLim)
                                metricRow("Pos gain", editor.posGain)
                                metricRow("Vel gain", editor.velGain)
                                metricRow("Vel I gain", editor.velIGain)
                                metricRow("Vel limit", editor.velLimit)
                                metricRow("Target tol", editor.targetToleranceTurns)
                                metricRow("Target vel tol", editor.targetVelToleranceTurnsS)
                                metricRow("Timeout s", editor.timeoutS)
                            }

                            if editor.moveMode == "trap_strict" {
                                LazyVGrid(columns: [GridItem(.adaptive(minimum: 110), spacing: 8)], spacing: 8) {
                                    metricRow("Trap vel", editor.trapVel)
                                    metricRow("Trap acc", editor.trapAcc)
                                    metricRow("Trap dec", editor.trapDec)
                                    metricRow("Settle s", editor.settleS)
                                }
                            } else if editor.moveMode == "mks_directional_direct" || editor.moveMode == "mks_directional_slew_direct" || editor.moveMode == "mks_directional_velocity_travel_direct" {
                                VStack(alignment: .leading, spacing: 6) {
                                    LazyVGrid(columns: [GridItem(.adaptive(minimum: 110), spacing: 8)], spacing: 8) {
                                        metricRow("Trap vel", editor.trapVel)
                                        metricRow("Trap acc", editor.trapAcc)
                                        metricRow("Trap dec", editor.trapDec)
                                    }
                                    Text("Trap fields are saved with the profile but are not used by direct MKS move modes.")
                                        .font(.caption)
                                        .foregroundStyle(.secondary)
                                }
                            }

                            if editor.moveMode == "mks_directional_direct" || editor.moveMode == "mks_directional_slew_direct" || editor.moveMode == "mks_directional_velocity_travel_direct" {
                                LazyVGrid(columns: [GridItem(.adaptive(minimum: 110), spacing: 8)], spacing: 8) {
                                    if !editor.preHoldS.isEmpty { metricRow("Pre hold s", editor.preHoldS) }
                                    if !editor.finalHoldS.isEmpty { metricRow("Final hold s", editor.finalHoldS) }
                                    if !editor.abortAbsTurns.isEmpty { metricRow("Abort abs", editor.abortAbsTurns) }
                                    metricRow("Fail to IDLE", editor.failToIdle ? "yes" : "no")
                                    metricRow("Reuse cal", editor.reuseExistingCalibration ? "yes" : "no")
                                    if !editor.polePairs.isEmpty { metricRow("Pole pairs", editor.polePairs) }
                                    if !editor.calibrationCurrent.isEmpty { metricRow("Motor cal A", editor.calibrationCurrent) }
                                    if !editor.encoderOffsetCalibrationCurrent.isEmpty { metricRow("Encoder cal A", editor.encoderOffsetCalibrationCurrent) }
                                }
                            }

                            if editor.moveMode == "mks_directional_slew_direct" || editor.moveMode == "mks_directional_velocity_travel_direct" {
                                LazyVGrid(columns: [GridItem(.adaptive(minimum: 110), spacing: 8)], spacing: 8) {
                                    if !editor.commandVelTurnsS.isEmpty { metricRow("Cmd vel t/s", editor.commandVelTurnsS) }
                                    if !editor.handoffWindowTurns.isEmpty { metricRow("Handoff turns", editor.handoffWindowTurns) }
                                    if !editor.commandDt.isEmpty { metricRow("Cmd dt s", editor.commandDt) }
                                    if !editor.travelPosGain.isEmpty { metricRow("Travel pos", editor.travelPosGain) }
                                    if !editor.travelVelGain.isEmpty { metricRow("Travel vel", editor.travelVelGain) }
                                    if !editor.travelVelIGain.isEmpty { metricRow("Travel vel I", editor.travelVelIGain) }
                                    if !editor.travelVelLimit.isEmpty { metricRow("Travel vel lim", editor.travelVelLimit) }
                                }
                            }
                        }
                        .padding(.top, 8)
                    } label: {
                        Text("Show detailed parameters")
                            .font(.subheadline.weight(.medium))
                    }
                } else {
                    Text("Loading detailed profile config…")
                        .foregroundStyle(.secondary)
                }

                if let limitations = detail?.limitations, !limitations.isEmpty {
                    DisclosureGroup(isExpanded: $showLimitations) {
                        VStack(alignment: .leading, spacing: 6) {
                            ForEach(limitations, id: \.self) { item in
                                Text("- \(item)")
                                    .foregroundStyle(.secondary)
                            }
                        }
                        .padding(.top, 6)
                    } label: {
                        Text("Show limitations")
                            .font(.subheadline.weight(.medium))
                    }
                }
            }
            .padding(12)
            .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
        }
    }
}

private struct MoveDiagnosticsCardView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @State private var showDetails = false

    private enum CandidateDiffState {
        case same
        case overridden
        case unavailable
    }

    private var resultObject: [String: JSONValue]? {
        vm.response?.result?.objectValue
    }

    private var moveObject: [String: JSONValue]? {
        guard let resultObject else { return nil }
        if let move = resultObject["move"]?.objectValue {
            return move
        }
        if let latest = resultObject["latest_result"]?.objectValue,
           let move = latest["move"]?.objectValue {
            return move
        }
        return nil
    }

    private var motionErrorObject: [String: JSONValue]? {
        resultObject?["error"]?.objectValue
    }

    private var motionActive: Bool {
        resultObject?["active"]?.boolValue ?? false
    }

    private var diagnostics: [String: JSONValue]? {
        moveObject?["travel_diagnostics"]?.objectValue
    }

    private var runtimeCandidate: [String: JSONValue]? {
        moveObject?["candidate"]?.objectValue
    }

    private var runtimeCandidatePreset: String? {
        moveObject?["candidate_preset"]?.stringValue
    }

    private var selectedEditor: ProfileEditorFormState? {
        vm.selectedProfileEditorLoaded ? vm.profileEditor : nil
    }

    private func number(_ key: String) -> Double? {
        diagnostics?[key]?.numberValue
    }

    private func text(_ key: String) -> String? {
        diagnostics?[key]?.stringValue
    }

    private func bool(_ key: String) -> Bool? {
        diagnostics?[key]?.boolValue
    }

    private func candidateNumber(_ key: String) -> Double? {
        runtimeCandidate?[key]?.numberValue
    }

    private func selectedCandidateNumber(_ key: String) -> Double? {
        guard let selectedEditor else { return nil }
        switch key {
        case "current_lim":
            return Double(selectedEditor.currentLim)
        case "pos_gain":
            return Double(selectedEditor.posGain)
        case "vel_gain":
            return Double(selectedEditor.velGain)
        case "vel_i_gain":
            return Double(selectedEditor.velIGain)
        case "vel_limit":
            return Double(selectedEditor.velLimit)
        default:
            return nil
        }
    }

    private func sameValue(_ lhs: Double?, _ rhs: Double?) -> Bool {
        guard let lhs, let rhs else { return false }
        return abs(lhs - rhs) <= 0.0005
    }

    private func candidateDiffState(_ key: String) -> CandidateDiffState {
        let selected = selectedCandidateNumber(key)
        let runtime = candidateNumber(key)
        if selected == nil || runtime == nil {
            return .unavailable
        }
        return sameValue(selected, runtime) ? .same : .overridden
    }

    private func diffLabel(_ state: CandidateDiffState) -> String {
        switch state {
        case .same:
            return "same"
        case .overridden:
            return "overridden"
        case .unavailable:
            return "unavailable"
        }
    }

    private func diffColor(_ state: CandidateDiffState) -> Color {
        switch state {
        case .same:
            return .green
        case .overridden:
            return .orange
        case .unavailable:
            return .gray
        }
    }

    private func formattedCandidateValue(_ value: Double?) -> String {
        guard let value else { return "n/a" }
        return String(format: "%.3f", value)
    }

    private func metricRow(_ title: String, _ value: String) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(title)
                .font(.caption)
                .foregroundStyle(.secondary)
            Text(value)
                .font(.system(.body, design: .monospaced))
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(10)
        .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 10, style: .continuous))
    }

    private func comparisonRow(_ title: String, key: String) -> some View {
        let selected = selectedCandidateNumber(key)
        let runtime = candidateNumber(key)
        let state = candidateDiffState(key)

        return VStack(alignment: .leading, spacing: 6) {
            HStack(alignment: .firstTextBaseline) {
                Text(title)
                    .font(.caption)
                    .foregroundStyle(.secondary)
                Spacer()
                Text(diffLabel(state))
                    .font(.caption2.weight(.semibold))
                    .padding(.horizontal, 8)
                    .padding(.vertical, 4)
                    .background(diffColor(state).opacity(0.14), in: Capsule())
            }
            Text("Selected \(formattedCandidateValue(selected))")
                .font(.system(.caption, design: .monospaced))
                .foregroundStyle(.secondary)
            Text("Runtime  \(formattedCandidateValue(runtime))")
                .font(.system(.body, design: .monospaced))
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(10)
        .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 10, style: .continuous))
    }

    private func classificationColor(_ value: String) -> Color {
        switch value {
        case "clean_travel":
            return .green
        case "mild_wave":
            return .yellow
        case "wavy_travel":
            return .orange
        case "hunting_travel", "faulted":
            return .red
        default:
            return .gray
        }
    }

    private func formattedTurnsPerSecond(_ value: Double?) -> String? {
        guard let value else { return nil }
        return String(format: "%.3f t/s", value)
    }

    private func formattedDegreesPerSecond(_ value: Double?) -> String? {
        guard let value else { return nil }
        return String(format: "%.1f deg/s", value)
    }

    private func formattedPercent(_ value: Double?) -> String? {
        guard let value else { return nil }
        return String(format: "%.0f%%", value * 100.0)
    }

    var body: some View {
        if let diagnostics {
            VStack(alignment: .leading, spacing: 10) {
                HStack {
                    Text("Latest Move Diagnostics")
                        .font(.headline)
                    Spacer()
                    if let classification = diagnostics["travel_classification"]?.stringValue {
                        Text(classification.replacingOccurrences(of: "_", with: " "))
                            .font(.caption.weight(.semibold))
                            .padding(.horizontal, 10)
                            .padding(.vertical, 6)
                            .background(classificationColor(classification).opacity(0.15), in: Capsule())
                    }
                }

                Text("Actual travel result from the last move. Use this to judge speed and shake, not just the selected profile numbers.")
                    .font(.caption)
                    .foregroundStyle(.secondary)

                if let message = motionErrorObject?["message"]?.stringValue, !message.isEmpty {
                    Text(message)
                        .font(.system(.caption, design: .monospaced))
                        .foregroundStyle(.secondary)
                }

                HStack(alignment: .top, spacing: 16) {
                    if let value = formattedTurnsPerSecond(number("achieved_avg_turns_s")) {
                        compactMoveStat("Achieved", value)
                    }
                    if let value = formattedTurnsPerSecond(number("commanded_turns_s")) {
                        compactMoveStat("Commanded", value)
                    }
                    if let value = formattedPercent(number("achieved_fraction_of_commanded")) {
                        compactMoveStat("Ratio", value)
                    }
                    if let value = formattedPercent(number("monotonic_fraction")) {
                        compactMoveStat("Monotonic", value)
                    }
                    if let err = number("final_error_abs_output_deg") {
                        compactMoveStat("Final err", String(format: "%.2f deg", err))
                    } else if let err = number("final_error_abs_turns") {
                        compactMoveStat("Final err", String(format: "%.4f t", err))
                    }
                    if let handoffReached = bool("handoff_reached") {
                        compactMoveStat("Handoff", handoffReached ? "reached" : "missed")
                    }
                    if let improvement = number("capture_improvement_turns") {
                        compactMoveStat("Capture Δ", String(format: "%.4f t", improvement))
                    }
                }

                DisclosureGroup(isExpanded: $showDetails) {
                    VStack(alignment: .leading, spacing: 10) {
                        LazyVGrid(columns: [GridItem(.adaptive(minimum: 130), spacing: 8)], spacing: 8) {
                            if let value = formattedTurnsPerSecond(number("commanded_turns_s")) {
                                metricRow("Commanded", value)
                            }
                            if let value = formattedTurnsPerSecond(number("achieved_avg_turns_s")) {
                                metricRow("Achieved avg", value)
                            }
                            if let value = formattedTurnsPerSecond(number("peak_turns_s")) {
                                metricRow("Peak", value)
                            }
                            if let value = formattedPercent(number("achieved_fraction_of_commanded")) {
                                metricRow("Achieved / cmd", value)
                            }
                            if let value = formattedPercent(number("monotonic_fraction")) {
                                metricRow("Monotonic", value)
                            }
                            if let backtrack = number("backtrack_turns") {
                                metricRow("Backtrack", String(format: "%.4f t", backtrack))
                            }
                            if let duration = number("duration_s") {
                                metricRow("Travel time", String(format: "%.3f s", duration))
                            }
                            if let err = number("final_error_abs_turns") {
                                metricRow("Final err", String(format: "%.4f t", err))
                            }
                            if let value = formattedDegreesPerSecond(number("commanded_output_deg_s")) {
                                metricRow("Cmd output", value)
                            }
                            if let value = formattedDegreesPerSecond(number("achieved_avg_output_deg_s")) {
                                metricRow("Avg output", value)
                            }
                            if let value = formattedDegreesPerSecond(number("peak_output_deg_s")) {
                                metricRow("Peak output", value)
                            }
                            if let err = number("final_error_abs_output_deg") {
                                metricRow("Final err out", String(format: "%.2f deg", err))
                            }
                            if let handoffReached = bool("handoff_reached") {
                                metricRow("Handoff", handoffReached ? "reached" : "missed")
                            }
                            if let handoffTime = number("handoff_time_s") {
                                metricRow("Handoff time", String(format: "%.3f s", handoffTime))
                            }
                            if let handoffError = number("handoff_error_turns") {
                                metricRow("Handoff err", String(format: "%+.4f t", handoffError))
                            }
                            if let failureStage = text("failure_stage") {
                                metricRow("Failure stage", failureStage)
                            }
                            if let phase = text("phase_summary") {
                                metricRow("Phase", phase)
                            }
                            if let startError = number("capture_start_error_turns") {
                                metricRow("Capture start", String(format: "%+.4f t", startError))
                            }
                            if let endError = number("capture_end_error_turns") {
                                metricRow("Capture end", String(format: "%+.4f t", endError))
                            }
                            if let improvement = number("capture_improvement_turns") {
                                metricRow("Capture Δ", String(format: "%.4f t", improvement))
                            }
                        }

                        if let classification = text("travel_classification"),
                           let achievedFraction = number("achieved_fraction_of_commanded"),
                           classification != "clean_travel" {
                            Text(
                                achievedFraction < 0.80
                                ? "Higher speed scales can be slower in practice when achieved travel speed drops below commanded speed due to hunting."
                                : "Travel quality is limited by shake/hunting rather than the commanded speed alone."
                            )
                            .font(.caption)
                            .foregroundStyle(.secondary)
                        }

                        if runtimeCandidate != nil || runtimeCandidatePreset != nil {
                            VStack(alignment: .leading, spacing: 8) {
                                HStack {
                                    Text("Effective Runtime Candidate")
                                        .font(.subheadline.weight(.semibold))
                                    Spacer()
                                    if let preset = runtimeCandidatePreset, !preset.isEmpty {
                                        Text(preset)
                                            .font(.caption.weight(.semibold))
                                            .padding(.horizontal, 10)
                                            .padding(.vertical, 6)
                                            .background(Color.blue.opacity(0.12), in: Capsule())
                                    }
                                }

                                Text("These are the actual gains and limits that drove the last move, after preset selection and any forked runtime overrides.")
                                    .font(.caption)
                                    .foregroundStyle(.secondary)

                                LazyVGrid(columns: [GridItem(.adaptive(minimum: 130), spacing: 8)], spacing: 8) {
                                    if let value = candidateNumber("current_lim") {
                                        metricRow("Run current A", String(format: "%.3f", value))
                                    }
                                    if let value = candidateNumber("pos_gain") {
                                        metricRow("Pos gain", String(format: "%.3f", value))
                                    }
                                    if let value = candidateNumber("vel_gain") {
                                        metricRow("Vel gain", String(format: "%.3f", value))
                                    }
                                    if let value = candidateNumber("vel_i_gain") {
                                        metricRow("Vel i gain", String(format: "%.3f", value))
                                    }
                                    if let value = candidateNumber("vel_limit") {
                                        metricRow("Vel limit", String(format: "%.3f", value))
                                    }
                                }

                                VStack(alignment: .leading, spacing: 8) {
                                    Text("Selected vs Runtime")
                                        .font(.subheadline.weight(.semibold))
                                    Text("This compares the selected profile editor values against the candidate that actually drove the last move.")
                                        .font(.caption)
                                        .foregroundStyle(.secondary)

                                    LazyVGrid(columns: [GridItem(.adaptive(minimum: 165), spacing: 8)], spacing: 8) {
                                        comparisonRow("Run current A", key: "current_lim")
                                        comparisonRow("Pos gain", key: "pos_gain")
                                        comparisonRow("Vel gain", key: "vel_gain")
                                        comparisonRow("Vel I gain", key: "vel_i_gain")
                                        comparisonRow("Vel limit", key: "vel_limit")
                                    }
                                }
                                .padding(.top, 2)
                            }
                        }
                    }
                    .padding(.top, 8)
                } label: {
                    Text("Show move diagnostics")
                        .font(.subheadline.weight(.medium))
                }
            }
            .padding(12)
            .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
        } else if let motionErrorObject {
            VStack(alignment: .leading, spacing: 10) {
                HStack {
                    Text("Latest Move Diagnostics")
                        .font(.headline)
                    Spacer()
                    Text("move failed")
                        .font(.caption.weight(.semibold))
                        .padding(.horizontal, 10)
                        .padding(.vertical, 6)
                        .background(Color.red.opacity(0.15), in: Capsule())
                }

                if let message = motionErrorObject["message"]?.stringValue, !message.isEmpty {
                    Text(message)
                        .font(.system(.body, design: .monospaced))
                        .textSelection(.enabled)
                } else {
                    Text("The last experimental move did not complete, so no completed-travel diagnostics were produced.")
                        .foregroundStyle(.secondary)
                }

                Text("This is already a dead-end signal for the current speed/move-law combination.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            .padding(12)
            .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
        } else if motionActive {
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Text("Latest Move Diagnostics")
                        .font(.headline)
                    Spacer()
                    ProgressView()
                        .controlSize(.small)
                }
                Text("Move in progress. Diagnostics appear when the move completes or fails.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            .padding(12)
            .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
        }
    }

    private func compactMoveStat(_ title: String, _ value: String) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(title)
                .font(.caption2)
                .foregroundStyle(.secondary)
            Text(value)
                .font(.system(.subheadline, design: .monospaced))
                .fontWeight(.medium)
        }
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

struct SyncAxisStatusCardView: View {
    let title: String
    let axisIndex: Int
    let serialNumber: String
    let snapshot: BackendSnapshot?
    let capabilities: BackendCapabilities?
    let zeroTurnsMotor: String

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack {
                Text(title)
                    .font(.headline)
                Spacer()
                Text(serialNumber.isEmpty ? "axis\(axisIndex)" : "\(serialNumber) / axis\(axisIndex)")
                    .font(.system(.caption, design: .monospaced))
                    .foregroundStyle(.secondary)
            }
            HStack {
                StatusBadge(
                    title: "Startup",
                    value: (capabilities?.startup_ready == true) ? "ready" : "not ready",
                    color: (capabilities?.startup_ready == true) ? .green : .orange
                )
                StatusBadge(
                    title: "State",
                    value: snapshot?.state.map(String.init) ?? "unknown",
                    color: (capabilities?.armed == true) ? .blue : .gray
                )
            }
            HStack {
                StatusBadge(
                    title: "Encoder",
                    value: (snapshot?.enc_ready == true) ? "ready" : "not ready",
                    color: (snapshot?.enc_ready == true) ? .green : .orange
                )
                StatusBadge(
                    title: "Pos",
                    value: snapshot?.pos_est.map { String(format: "%.4f t", $0) } ?? "unknown",
                    color: .gray
                )
            }
            Text("Zero anchor: \(zeroTurnsMotor.isEmpty ? "not captured" : zeroTurnsMotor)")
                .font(.system(.caption, design: .monospaced))
                .foregroundStyle(.secondary)
        }
        .padding(12)
        .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
    }
}

struct DualAxisSyncSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    var body: some View {
        VStack(alignment: .leading, spacing: 14) {
            HStack {
                Text("Dual-Joint Sync Move")
                    .font(.title2.bold())
                Spacer()
                Button("Detect Boards") {
                    Task { await vm.discoverAndFillSyncBoardSerials() }
                }
                .disabled(vm.isBusy)
                Button("Refresh A/B Status") {
                    Task { await vm.refreshSyncAxesStatus() }
                }
                .disabled(vm.isBusy)
            }

            Text("Safe first step for two joints at once: same-time single-target move on axis A and axis B with independent board serials and independent profiles. This is not coordinated kinematics. Use 'Detect Boards' to autofill the connected board serials; if only one runtime board is found, both serial fields are set to that same board.")
                .foregroundStyle(.secondary)

            HStack(alignment: .top, spacing: 12) {
                VStack(alignment: .leading, spacing: 8) {
                    Text("Axis Pair")
                        .font(.headline)
                    Stepper("Axis A: \(vm.syncMoveForm.axisAIndex)", value: $vm.syncMoveForm.axisAIndex, in: 0...1)
                    Stepper("Axis B: \(vm.syncMoveForm.axisBIndex)", value: $vm.syncMoveForm.axisBIndex, in: 0...1)
                    LabeledInputField(title: "Board A serial (optional)", text: $vm.syncMoveForm.serialA)
                    LabeledInputField(title: "Board B serial (optional)", text: $vm.syncMoveForm.serialB)
                }
                VStack(alignment: .leading, spacing: 8) {
                    Text("Profiles")
                        .font(.headline)
                    Picker("Sync profile default", selection: $vm.syncMoveForm.profileName) {
                        ForEach(vm.profiles, id: \.self) { profile in
                            Text(profile).tag(profile)
                        }
                    }
                    .disabled(vm.profiles.isEmpty)
                    Picker("Profile A", selection: $vm.syncMoveForm.profileAName) {
                        ForEach(vm.profiles, id: \.self) { profile in
                            Text(profile).tag(profile)
                        }
                    }
                    .disabled(vm.profiles.isEmpty)
                    Picker("Profile B", selection: $vm.syncMoveForm.profileBName) {
                        ForEach(vm.profiles, id: \.self) { profile in
                            Text(profile).tag(profile)
                        }
                    }
                    .disabled(vm.profiles.isEmpty)
                    Picker("Angle space", selection: $vm.syncMoveForm.angleSpace) {
                        Text("Gearbox output").tag("gearbox_output")
                        Text("Motor").tag("motor")
                    }
                    .pickerStyle(.segmented)
                    Toggle("Release both to IDLE after move", isOn: $vm.syncMoveForm.releaseAfterMove)
                }
            }

            HStack(alignment: .top, spacing: 12) {
                SyncAxisStatusCardView(
                    title: "Axis A",
                    axisIndex: vm.syncMoveForm.axisAIndex,
                    serialNumber: vm.syncMoveForm.serialA,
                    snapshot: vm.syncAxisASnapshot,
                    capabilities: vm.syncAxisACapabilities,
                    zeroTurnsMotor: vm.syncMoveForm.zeroATurnsMotor
                )
                SyncAxisStatusCardView(
                    title: "Axis B",
                    axisIndex: vm.syncMoveForm.axisBIndex,
                    serialNumber: vm.syncMoveForm.serialB,
                    snapshot: vm.syncAxisBSnapshot,
                    capabilities: vm.syncAxisBCapabilities,
                    zeroTurnsMotor: vm.syncMoveForm.zeroBTurnsMotor
                )
            }

            HStack(alignment: .top, spacing: 12) {
                VStack(alignment: .leading, spacing: 10) {
                    Text("Axis A Target")
                        .font(.headline)
                    LabeledInputField(title: "Angle A (deg)", text: $vm.syncMoveForm.angleADeg)
                    LabeledInputField(title: "Gear ratio A", text: $vm.syncMoveForm.gearRatioA)
                    LabeledInputField(title: "Zero A turns motor", text: $vm.syncMoveForm.zeroATurnsMotor)
                    Button("Capture A current as zero") {
                        Task { await vm.captureSyncZero(axisRole: "a") }
                    }
                    .disabled(vm.isBusy)
                }
                VStack(alignment: .leading, spacing: 10) {
                    Text("Axis B Target")
                        .font(.headline)
                    LabeledInputField(title: "Angle B (deg)", text: $vm.syncMoveForm.angleBDeg)
                    LabeledInputField(title: "Gear ratio B", text: $vm.syncMoveForm.gearRatioB)
                    LabeledInputField(title: "Zero B turns motor", text: $vm.syncMoveForm.zeroBTurnsMotor)
                    Button("Capture B current as zero") {
                        Task { await vm.captureSyncZero(axisRole: "b") }
                    }
                    .disabled(vm.isBusy)
                }
                VStack(alignment: .leading, spacing: 10) {
                    Text("Timing")
                        .font(.headline)
                    LabeledInputField(title: "Timeout (s, optional)", text: $vm.syncMoveForm.timeoutSeconds)
                    Button("Run Synced 2-Axis Move") {
                        Task { await vm.moveTwoAxesSynced() }
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(vm.syncMoveDisabledReason != nil)
                    if let disabledReason = vm.syncMoveDisabledReason {
                        Text(disabledReason)
                            .font(.system(.caption, design: .monospaced))
                            .foregroundStyle(.secondary)
                            .fixedSize(horizontal: false, vertical: true)
                    }
                }
            }

            if let result = vm.syncResultResponse {
                VStack(alignment: .leading, spacing: 6) {
                    Text("Latest Sync Result")
                        .font(.headline)
                    Text(result.resultSummary)
                        .font(.system(.body, design: .monospaced))
                        .textSelection(.enabled)
                }
                .padding(12)
                .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
            }
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
    @AppStorage("FOCUI.visiblePanels") private var visiblePanelsRaw = ConsolePanel.defaultVisibleRaw

    private var visiblePanels: Set<String> {
        let ids = visiblePanelsRaw
            .split(separator: ",")
            .map { String($0).trimmingCharacters(in: .whitespacesAndNewlines) }
            .filter { !$0.isEmpty }
        let allowed = Set(ConsolePanel.allCases.map(\.rawValue))
        let filtered = Set(ids).intersection(allowed)
        return filtered.isEmpty ? Set(ConsolePanel.allCases.map(\.rawValue)) : filtered
    }

    private func persistVisiblePanels(_ panels: Set<String>) {
        let ordered = ConsolePanel.allCases.map(\.rawValue).filter { panels.contains($0) }
        visiblePanelsRaw = ordered.isEmpty ? ConsolePanel.defaultVisibleRaw : ordered.joined(separator: ",")
    }

    private func togglePanel(_ panel: ConsolePanel) {
        var next = visiblePanels
        if next.contains(panel.rawValue) {
            if next.count == 1 {
                return
            }
            next.remove(panel.rawValue)
        } else {
            next.insert(panel.rawValue)
        }
        persistVisiblePanels(next)
    }

    private func isPanelVisible(_ panel: ConsolePanel) -> Bool {
        visiblePanels.contains(panel.rawValue)
    }

    var body: some View {
        HSplitView {
            BackendSidebarView(
                vm: vm,
                visiblePanels: visiblePanels,
                togglePanel: togglePanel,
                showAllPanels: { persistVisiblePanels(Set(ConsolePanel.allCases.map(\.rawValue))) }
            )

            ScrollView {
                VStack(alignment: .leading, spacing: 18) {
                    header
                    if isPanelVisible(.stateAtGlance) {
                        readinessGrid
                    }
                    if isPanelVisible(.outputSensor) {
                        outputSensorLaunchWarning
                        outputSensorSection
                    }
                    if isPanelVisible(.guidedBringup) {
                        guidedBringupSection
                    }
                    if isPanelVisible(.telemetry) {
                        telemetrySection
                    }
                    if isPanelVisible(.dualAxisSync) {
                        dualAxisSyncSection
                    }
                    if isPanelVisible(.move) {
                        moveSection
                    }
                    if isPanelVisible(.sliderFollow) {
                        sliderFollowSection
                    }
                    if isPanelVisible(.diagnosis) {
                        diagnosisSection
                    }
                    if isPanelVisible(.factSheet) {
                        factSheetSection
                    }
                    if isPanelVisible(.backendResult) {
                        rawResultSection
                    }
                }
                .padding(20)
            }
        }
        .frame(minWidth: 1180, minHeight: 860)
        .task {
            await vm.ensureInitialRefresh()
            await vm.refreshSyncAxesStatus()
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

    private var guidedBringupSection: some View {
        GuidedBringupSectionView(vm: vm)
    }

    private var readinessGrid: some View {
        ReadinessGridView(
            vm: vm,
            live: vm.liveMonitor,
            diagnosis: vm.diagnosis,
            fallbackSnapshot: vm.response?.snapshot,
            fallbackDevice: vm.response?.device
        )
    }

    @ViewBuilder
    private var outputSensorSection: some View {
        if vm.outputSensor?.configured == true {
            OutputSensorSectionView(vm: vm)
        }
    }

    @ViewBuilder
    private var outputSensorLaunchWarning: some View {
        if vm.outputSensorPortEnv == nil {
            OutputSensorLaunchWarningView()
        }
    }

    private var moveSection: some View {
        MoveSectionView(vm: vm, live: vm.liveMonitor)
    }

    private var dualAxisSyncSection: some View {
        DualAxisSyncSectionView(vm: vm)
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
