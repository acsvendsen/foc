import SwiftUI
import Combine

@MainActor
final class OperatorConsoleViewModel: ObservableObject {
    @Published var repoRoot: String
    @Published var axisIndex: Int = 0
    @Published var kvEstimate: String = "140"
    @Published var lineLineROhm: String = "0.30"
    @Published var settleSeconds: String = "0.15"
    @Published var debugMode: Bool = false

    @Published var moveForm = MoveFormState()
    @Published var response: BackendResponse?
    @Published var isBusy = false
    @Published var lastClientError: String?

    private let backend = BackendClient()

    init() {
        self.repoRoot = backend.detectRepoRoot()
    }

    var capabilities: BackendCapabilities? { response?.capabilities }
    var diagnosis: BackendDiagnosis? { response?.diagnosis }
    var snapshot: BackendSnapshot? { response?.snapshot }
    var profiles: [String] { response?.available_profiles ?? [] }
    var profileDetails: [BackendProfileDetail] { response?.available_profile_details ?? [] }

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

    func moveContinuous() async {
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
        await run(action: "move-continuous", arguments: args)
    }

    private func run(action: String, arguments: [String] = []) async {
        isBusy = true
        lastClientError = nil
        defer { isBusy = false }
        do {
            let context = BackendClient.RequestContext(
                repoRoot: repoRoot,
                axisIndex: axisIndex,
                kvEstimate: kvEstimate,
                lineLineROhm: lineLineROhm,
                settleSeconds: settleSeconds,
                debug: debugMode
            )
            let result = try await backend.run(action: action, arguments: arguments, context: context)
            if moveForm.profileName.isEmpty, let first = result.available_profiles?.first {
                moveForm.profileName = first
            }
            if profiles.contains(moveForm.profileName) == false, let first = result.available_profiles?.first {
                moveForm.profileName = first
            }
            response = result
        } catch {
            lastClientError = error.localizedDescription
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
            ScrollView {
                Form {
                    Section("Backend") {
                        TextField("Repo root", text: $vm.repoRoot)
                        Stepper("Axis \(vm.axisIndex)", value: $vm.axisIndex, in: 0...1)
                        TextField("KV estimate", text: $vm.kvEstimate)
                        TextField("Line-line resistance (ohm)", text: $vm.lineLineROhm)
                        TextField("Clear-errors settle (s)", text: $vm.settleSeconds)
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

            ScrollView {
                VStack(alignment: .leading, spacing: 18) {
                    header
                    readinessGrid
                    moveSection
                    diagnosisSection
                    factSheetSection
                    rawResultSection
                }
                .padding(20)
            }
        }
        .frame(minWidth: 1180, minHeight: 860)
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

    private var readinessGrid: some View {
        let startupColor: Color = (vm.capabilities?.startup_ready == true) ? .green : .orange
        let errorColor: Color = (vm.capabilities?.has_latched_errors == true) ? .red : .green
        let armedColor: Color = (vm.capabilities?.armed == true) ? .blue : .gray
        let idleColor: Color = (vm.capabilities?.idle == true) ? .green : .gray
        return VStack(alignment: .leading, spacing: 12) {
            Text("State at a Glance")
                .font(.title2.bold())
            LazyVGrid(columns: [GridItem(.adaptive(minimum: 220), spacing: 12)], spacing: 12) {
                StatusBadge(title: "Verdict", value: vm.diagnosis?.verdict ?? "unknown", color: startupColor)
                StatusBadge(title: "Startup", value: (vm.capabilities?.startup_ready == true) ? "ready" : "not ready", color: startupColor)
                StatusBadge(title: "Latched Errors", value: (vm.capabilities?.has_latched_errors == true) ? "present" : "none", color: errorColor)
                StatusBadge(title: "Axis State", value: vm.snapshot?.state.map(String.init) ?? "unknown", color: armedColor)
                StatusBadge(title: "Armed", value: (vm.capabilities?.armed == true) ? "closed-loop" : "not armed", color: armedColor)
                StatusBadge(title: "Idle", value: (vm.capabilities?.idle == true) ? "idle" : "not idle", color: idleColor)
                StatusBadge(title: "Encoder Ready", value: (vm.snapshot?.enc_ready == true) ? "true" : "false", color: (vm.snapshot?.enc_ready == true) ? .green : .orange)
                StatusBadge(title: "Index Found", value: (vm.snapshot?.enc_index_found == true) ? "true" : "false", color: (vm.snapshot?.enc_index_found == true) ? .green : .orange)
                StatusBadge(title: "Pos Estimate", value: vm.snapshot?.pos_est.map { String(format: "%.6f t", $0) } ?? "unknown", color: .gray)
                StatusBadge(title: "Vbus", value: vm.response?.device?.vbus_voltage.map { String(format: "%.2f V", $0) } ?? "unknown", color: .gray)
            }
        }
    }

    private var moveSection: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Continuous Move")
                .font(.title2.bold())
            Text("Only the validated continuous move path is exposed here. No segmented waypoint mode.")
                .foregroundStyle(.secondary)

            HStack(spacing: 12) {
                TextField("Angle (deg)", text: $vm.moveForm.angleDeg)
                TextField("Gear ratio", text: $vm.moveForm.gearRatio)
                TextField("Timeout (s, optional)", text: $vm.moveForm.timeoutSeconds)
            }
            Picker("Angle space", selection: $vm.moveForm.angleSpace) {
                Text("Gearbox output").tag("gearbox_output")
                Text("Motor").tag("motor")
            }
            .pickerStyle(.segmented)

            Toggle("Relative to current", isOn: $vm.moveForm.relativeToCurrent)
            HStack(spacing: 12) {
                TextField("Zero turns motor (absolute mode)", text: $vm.moveForm.zeroTurnsMotor)
                    .disabled(vm.moveForm.relativeToCurrent)
                Button("Capture Current as Zero") {
                    vm.captureZeroHere()
                }
                .disabled(vm.isBusy || vm.capabilities?.can_capture_zero_here == false)
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

            Button("Run Continuous Move") {
                Task { await vm.moveContinuous() }
            }
            .buttonStyle(.borderedProminent)
            .disabled(vm.isBusy || vm.capabilities?.can_move_continuous != true)
        }
        .padding(16)
        .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
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
                TextEditor(text: .constant(rawJSON))
                    .font(.system(.caption, design: .monospaced))
                    .frame(minHeight: 220)
                    .border(Color.secondary.opacity(0.25))
            }
        }
        .padding(16)
        .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
    }
}
