import SwiftUI
import Combine

// MARK: - Reusable Components

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


// MARK: - Auto Direction Result

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

// MARK: - Telemetry

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

// MARK: - Readiness

struct ReadinessGridView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    private var capabilities: BackendCapabilities? { vm.capabilities }
    private var snapshot: BackendSnapshot? { vm.snapshot }
    private var device: BackendDevice? { vm.boardDevice }
    private var diagnosis: BackendDiagnosis? { vm.diagnosis }

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

// MARK: - Output Sensor

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

// MARK: - Guided Bring-Up

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

// MARK: - Profile Editor & Tuning

private struct SlewRuntimeTuningView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @AppStorage("FOCUI.profileEditorWorkspaceExpanded") private var isExpanded = true

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

private func isStartupSensitiveDirectProfileMoveMode(_ moveMode: String) -> Bool {
    switch profileStrategyChoice(for: moveMode) {
    case .directPosition, .directionalDirect, .directionalSlew, .directionalVelocityTravel, .velocityPointToPoint, .directionalTorqueTravel:
        return true
    case .trapStrict:
        return false
    }
}

private struct ProfileDriveModeButton: View {
    let title: String
    let subtitle: String
    let selected: Bool
    let disabled: Bool
    let action: () -> Void

    var body: some View {
        Button(action: action) {
            VStack(alignment: .leading, spacing: 4) {
                Text(title)
                    .font(.caption.weight(.semibold))
                Text(subtitle)
                    .font(.caption2)
                    .foregroundStyle(.secondary)
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(10)
            .background(
                (selected ? Color.accentColor.opacity(disabled ? 0.08 : 0.16) : Color(nsColor: .textBackgroundColor)),
                in: RoundedRectangle(cornerRadius: 10, style: .continuous)
            )
            .overlay(
                RoundedRectangle(cornerRadius: 10, style: .continuous)
                    .stroke(selected ? Color.accentColor : Color.gray.opacity(0.15), lineWidth: selected ? 1.5 : 1)
            )
        }
        .buttonStyle(.plain)
        .disabled(disabled)
        .opacity(disabled ? 0.55 : 1.0)
    }
}

private struct ProfileStrategyChip: View {
    let title: String
    let selected: Bool
    let action: () -> Void

    var body: some View {
        Button(action: action) {
            Text(title)
                .font(.caption.weight(.semibold))
                .padding(.horizontal, 10)
                .padding(.vertical, 8)
                .background(selected ? Color.accentColor.opacity(0.16) : Color(nsColor: .textBackgroundColor), in: Capsule())
                .overlay(
                    Capsule()
                        .stroke(selected ? Color.accentColor : Color.gray.opacity(0.15), lineWidth: selected ? 1.5 : 1)
                )
        }
        .buttonStyle(.plain)
    }
}

struct ProfileEditorSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @AppStorage("FOCUI.profileEditorWorkspaceExpanded") private var isExpanded = true

    private var moveMode: String { vm.profileEditor.moveMode.trimmingCharacters(in: .whitespacesAndNewlines).lowercased() }
    private var isTrapProfile: Bool { moveMode == "trap_strict" || moveMode.isEmpty }
    private var isDirectProfile: Bool { isDirectProfileMoveMode(moveMode) }
    private var isShapedTravelProfile: Bool { isShapedTravelProfileMoveMode(moveMode) }
    private var isTorqueTravelProfile: Bool { moveMode == ProfileDriveStrategyChoice.directionalTorqueTravel.rawValue }
    private var isReadOnlyBuiltIn: Bool { vm.profileEditor.isBuiltInReadOnly }
    private var selectedDrivePrimitive: ProfileDrivePrimitive { profileDrivePrimitive(for: moveMode) }
    private var selectedStrategyChoice: ProfileDriveStrategyChoice { profileStrategyChoice(for: moveMode) }

    private var availableStrategyChoices: [ProfileDriveStrategyChoice] {
        switch selectedDrivePrimitive {
        case .positionLed:
            return [.trapStrict, .directPosition, .directionalDirect, .directionalSlew]
        case .velocityLed:
            return [.directionalVelocityTravel, .velocityPointToPoint]
        case .torqueCurrentLed:
            return [.directionalTorqueTravel]
        }
    }

    private func selectDrivePrimitive(_ primitive: ProfileDrivePrimitive) {
        switch primitive {
        case .positionLed:
            if selectedDrivePrimitive != .positionLed {
                applyDriveStrategy(.directionalDirect)
            }
        case .velocityLed:
            if selectedDrivePrimitive != .velocityLed {
                applyDriveStrategy(.directionalVelocityTravel)
            }
        case .torqueCurrentLed:
            if selectedDrivePrimitive != .torqueCurrentLed {
                applyDriveStrategy(.directionalTorqueTravel)
            }
        }
    }

    private func applyDriveStrategy(_ strategy: ProfileDriveStrategyChoice) {
        vm.profileEditor.moveMode = strategy.rawValue
    }

    private func applyStrategyBaseline(_ strategy: ProfileDriveStrategyChoice) {
        vm.profileEditor.moveMode = strategy.rawValue
        switch strategy {
        case .trapStrict:
            vm.profileEditor.trapVel = "0.28"
            vm.profileEditor.trapAcc = "0.32"
            vm.profileEditor.trapDec = "0.32"
            vm.profileEditor.targetToleranceTurns = "0.03"
            vm.profileEditor.targetVelToleranceTurnsS = "0.20"
            vm.profileEditor.timeoutS = "8.0"
            vm.profileEditor.settleS = "0.08"
        case .directPosition:
            vm.profileEditor.finalHoldS = "0.90"
            vm.profileEditor.abortAbsTurns = "0.90"
            vm.profileEditor.targetToleranceTurns = "0.03"
            vm.profileEditor.targetVelToleranceTurnsS = "0.20"
            vm.profileEditor.timeoutS = "8.0"
        case .directionalDirect:
            vm.profileEditor.preHoldS = "0.70"
            vm.profileEditor.finalHoldS = "0.90"
            vm.profileEditor.abortAbsTurns = "0.90"
            vm.profileEditor.targetToleranceTurns = "0.03"
            vm.profileEditor.targetVelToleranceTurnsS = "0.20"
            vm.profileEditor.timeoutS = "8.0"
        case .directionalSlew:
            vm.profileEditor.preHoldS = "0.25"
            vm.profileEditor.finalHoldS = "0.90"
            vm.profileEditor.abortAbsTurns = "3.00"
            vm.profileEditor.commandVelTurnsS = "0.30"
            vm.profileEditor.handoffWindowTurns = "0.10"
            vm.profileEditor.commandDt = "0.01"
            vm.profileEditor.travelPosGain = vm.profileEditor.posGain
            vm.profileEditor.travelVelGain = vm.profileEditor.velGain
            vm.profileEditor.travelVelIGain = vm.profileEditor.velIGain
            vm.profileEditor.travelVelLimit = vm.profileEditor.velLimit
        case .directionalVelocityTravel:
            vm.profileEditor.preHoldS = "0.10"
            vm.profileEditor.finalHoldS = "0.90"
            vm.profileEditor.abortAbsTurns = "3.00"
            vm.profileEditor.commandVelTurnsS = "4.00"
            vm.profileEditor.handoffWindowTurns = "0.35"
            vm.profileEditor.commandDt = "0.01"
        case .velocityPointToPoint:
            vm.profileEditor.finalHoldS = "0.90"
            vm.profileEditor.abortAbsTurns = "3.00"
            vm.profileEditor.commandVelTurnsS = "1.00"
            vm.profileEditor.handoffWindowTurns = "0.20"
            vm.profileEditor.commandDt = "0.01"
        case .directionalTorqueTravel:
            vm.profileEditor.preHoldS = "0.10"
            vm.profileEditor.finalHoldS = "0.90"
            vm.profileEditor.abortAbsTurns = "3.00"
            vm.profileEditor.commandTorqueNm = "0.12"
            vm.profileEditor.stictionKickNm = "0.18"
            vm.profileEditor.kickDurationS = "0.08"
            vm.profileEditor.handoffWindowTurns = "0.15"
            vm.profileEditor.commandDt = "0.01"
            vm.profileEditor.velAbortTurnsS = "1.50"
        }
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            DisclosureGroup(isExpanded: $isExpanded) {
                VStack(alignment: .leading, spacing: 12) {
                    HStack(alignment: .top, spacing: 16) {
                        ControlMethodStat(title: "Staged payload", value: vm.profileEditor.name.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty ? "unsaved" : vm.profileEditor.name)
                        ControlMethodStat(title: "Loaded from", value: vm.profileEditor.loadedProfileName.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty ? "none" : vm.profileEditor.loadedProfileName)
                        ControlMethodStat(title: "Primitive", value: selectedDrivePrimitive.rawValue)
                        ControlMethodStat(title: "Strategy", value: selectedStrategyChoice.label)
                    }

                    if isReadOnlyBuiltIn {
                        HStack(alignment: .top, spacing: 12) {
                            VStack(alignment: .leading, spacing: 4) {
                                Text("Built-in profile: read-only")
                                    .font(.subheadline.weight(.semibold))
                                Text("You can tune the knobs below as a working copy. Saving will automatically fork this built-in into a manual profile instead of shadowing the built-in entry.")
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                            }
                            Spacer()
                            Button("Start Manual Copy Now") {
                                vm.forkLoadedProfileEditor()
                                isExpanded = true
                            }
                            .buttonStyle(.borderedProminent)
                        }
                    } else {
                        Text("Runs use the editor payload below, not whatever is merely highlighted elsewhere. Save with the same name to overwrite it, or change the name to fork a new manual profile. Saved manual profiles show up in the manual-profile dropdown above, filtered by the selected drive primitive.")
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
                            Text("Drive Strategy")
                                .font(.headline)
                            Text("Choose the board-side control primitive this saved continuous profile will actually use. This must match the strategy you intend to test, otherwise the numbers in the editor will lie about what the motor is doing.")
                                .font(.caption)
                                .foregroundStyle(.secondary)

                            HStack(alignment: .top, spacing: 12) {
                                ProfileDriveModeButton(
                                    title: "Position-led",
                                    subtitle: "Board is told where the motor should land. Better for classic trap or direct-settle profiles.",
                                    selected: selectedDrivePrimitive == .positionLed,
                                    disabled: false
                                ) {
                                    selectDrivePrimitive(.positionLed)
                                }
                                ProfileDriveModeButton(
                                    title: "Velocity-led",
                                    subtitle: "Board is told how fast to travel, then capture happens near target. Better when position travel hunts.",
                                    selected: selectedDrivePrimitive == .velocityLed,
                                    disabled: false
                                ) {
                                    selectDrivePrimitive(.velocityLed)
                                }
                                ProfileDriveModeButton(
                                    title: "Current / torque-led",
                                    subtitle: "Board pushes with a travel torque ceiling, then hands off to direct position capture near target. Experimental and bounded.",
                                    selected: selectedDrivePrimitive == .torqueCurrentLed,
                                    disabled: false
                                ) {
                                    selectDrivePrimitive(.torqueCurrentLed)
                                }
                            }

                            Text("Saved profile primitive: \(selectedDrivePrimitive.rawValue). Actual saved move mode: `\(vm.profileEditor.moveMode)`.")
                                .font(.caption)
                                .foregroundStyle(.secondary)

                            if !availableStrategyChoices.isEmpty {
                                VStack(alignment: .leading, spacing: 8) {
                                    Text("Strategy within \(selectedDrivePrimitive.rawValue)")
                                        .font(.subheadline.weight(.medium))
                                    LazyVGrid(columns: [GridItem(.adaptive(minimum: 180), spacing: 8)], alignment: .leading, spacing: 8) {
                                        ForEach(availableStrategyChoices) { choice in
                                            ProfileStrategyChip(
                                                title: choice.label,
                                                selected: selectedStrategyChoice == choice
                                            ) {
                                                applyDriveStrategy(choice)
                                            }
                                        }
                                    }
                                    Text(selectedStrategyChoice.summary)
                                        .font(.caption)
                                        .foregroundStyle(.secondary)
                                    HStack(alignment: .top, spacing: 16) {
                                        ControlMethodStat(title: "Board primitive", value: selectedStrategyChoice.boardPrimitive.rawValue)
                                        ControlMethodStat(title: "Saved move_mode", value: vm.profileEditor.moveMode)
                                        ControlMethodStat(title: "Uses output encoder", value: "No")
                                        ControlMethodStat(title: "Key fields", value: selectedStrategyChoice.keyFields.joined(separator: ", "))
                                    }
                                    Button("Apply Baseline for Selected Strategy") {
                                        applyStrategyBaseline(selectedStrategyChoice)
                                    }
                                    .buttonStyle(.bordered)
                                    Text("This writes a sane baseline into the fields below for the selected strategy. It is a starting point, not truth. If the plant disagrees, the plant wins.")
                                        .font(.caption)
                                        .foregroundStyle(.secondary)
                                }
                            }
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
                                Text(isTorqueTravelProfile ? "Torque Travel Overrides" : "Travel Overrides")
                                    .font(.headline)
                                Text(
                                    isTorqueTravelProfile
                                    ? "These fields define the bounded torque-led travel phase before direct capture takes over. `Travel torque Nm` is the sustained travel push. `Kick torque Nm` is only for breakaway. `Vel abort t/s` is a hard safety cutoff."
                                    : moveMode == "mks_velocity_point_to_point_direct"
                                    ? "These apply only during the velocity-led travel phase. `Handoff turns` is the distance from target where velocity mode yields to final position capture."
                                    : "These apply only during the travel phase. Final settle reverts to the base gains above."
                                )
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                                if isTorqueTravelProfile {
                                    HStack(alignment: .top, spacing: 12) {
                                        LabeledInputField(title: "Travel torque Nm", text: $vm.profileEditor.commandTorqueNm)
                                        LabeledInputField(title: "Kick torque Nm", text: $vm.profileEditor.stictionKickNm)
                                        LabeledInputField(title: "Kick duration s", text: $vm.profileEditor.kickDurationS)
                                    }
                                    HStack(alignment: .top, spacing: 12) {
                                        LabeledInputField(title: "Handoff turns", text: $vm.profileEditor.handoffWindowTurns)
                                        LabeledInputField(title: "Cmd dt s", text: $vm.profileEditor.commandDt)
                                        LabeledInputField(title: "Vel abort t/s", text: $vm.profileEditor.velAbortTurnsS)
                                    }
                                } else {
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
                .padding(.top, 8)
            } label: {
                HStack {
                    VStack(alignment: .leading, spacing: 2) {
                        Text("Manual Tuning Workspace")
                            .font(.title3.bold())
                        Text(isReadOnlyBuiltIn ? "Built-in profiles are read-only until forked. Fork first, then these knobs become your editable manual workspace." : "These are the manual knobs for the staged run payload. Change them here, run, and save to store the variant in the manual-profile list.")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                    }
                    Spacer()
                    Button("Reload Selected") {
                        Task { await vm.loadProfileEditor() }
                    }
                    .disabled(vm.moveForm.profileName.isEmpty)
                    Button(isReadOnlyBuiltIn ? "Save as Manual Profile" : "Save Manual Profile") {
                        if isReadOnlyBuiltIn {
                            Task { await vm.saveProfileEditor() }
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

// MARK: - Move Section

struct MoveSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @AppStorage("FOCUI.showDiagnosticControlMethods") private var showDiagnosticControlMethods = false

    private var stagedProfileName: String {
        let staged = vm.profileEditor.name.trimmingCharacters(in: .whitespacesAndNewlines)
        return staged.isEmpty ? vm.moveForm.profileName : staged
    }

    private var capabilities: BackendCapabilities? { vm.capabilities }
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

            ControlMethodCard(
                title: "Profile Workspace",
                summary: "Browsing a template or saved manual profile does nothing by itself. The editor below is the truth source for the next continuous move, and only an explicit load action is allowed to replace it.",
                badgeText: vm.selectedProfileBrowsePrimitive.rawValue,
                badgeTint: .blue
            ) {
                VStack(alignment: .leading, spacing: 10) {
                    HStack(alignment: .top, spacing: 16) {
                        ControlMethodStat(title: "Loaded profile", value: vm.moveForm.profileName)
                        ControlMethodStat(title: "Staged run payload", value: stagedProfileName)
                        ControlMethodStat(title: "Drive primitive", value: vm.selectedProfileBrowsePrimitive.rawValue)
                    }

                    Picker(
                        "Drive primitive",
                        selection: Binding(
                            get: { vm.moveForm.profileBrowsePrimitive },
                            set: { newValue in
                                if let primitive = ProfileDrivePrimitive(rawValue: newValue) {
                                    vm.setProfileBrowsePrimitive(primitive)
                                } else {
                                    vm.moveForm.profileBrowsePrimitive = newValue
                                }
                            }
                        )
                    ) {
                        ForEach(ProfileDrivePrimitive.allCases) { primitive in
                            Text(primitive.rawValue).tag(primitive.rawValue)
                        }
                    }
                    .pickerStyle(.segmented)

                    VStack(alignment: .leading, spacing: 8) {
                        HStack(alignment: .top, spacing: 12) {
                            Picker("Built-in templates", selection: $vm.moveForm.templateProfileSelection) {
                                Text("None").tag("")
                                ForEach(vm.filteredTemplateProfiles) { detail in
                                    Text(detail.name).tag(detail.name)
                                }
                            }
                            .frame(maxWidth: .infinity, alignment: .leading)
                            Button("Load Template into Editor") {
                                Task { await vm.loadSelectedTemplateProfile() }
                            }
                            .disabled(vm.isBusy || vm.moveForm.templateProfileSelection.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty)
                        }
                        if vm.filteredTemplateProfiles.isEmpty {
                            Text("No built-in templates match the currently selected drive primitive.")
                                .font(.caption)
                                .foregroundStyle(.secondary)
                        }
                    }

                    VStack(alignment: .leading, spacing: 8) {
                        HStack(alignment: .top, spacing: 12) {
                            Picker("Saved manual profiles", selection: $vm.moveForm.manualProfileSelection) {
                                Text("None").tag("")
                                ForEach(vm.filteredManualProfiles) { detail in
                                    Text(detail.name).tag(detail.name)
                                }
                            }
                            .frame(maxWidth: .infinity, alignment: .leading)
                            Button("Load Saved Manual") {
                                Task { await vm.loadSelectedManualProfile() }
                            }
                            .disabled(vm.isBusy || vm.moveForm.manualProfileSelection.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty)
                        }
                        if vm.filteredManualProfiles.isEmpty {
                            Text("No saved manual profiles match the currently selected drive primitive yet. Save one from the editor below to populate this list.")
                                .font(.caption)
                                .foregroundStyle(.secondary)
                        }
                    }

                    Text("Filtering is strict: templates and saved manual profiles that do not match the chosen board-side primitive are intentionally hidden so the UI cannot imply a different drive mode than the one you are editing.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }

            SelectedProfileSummaryView(vm: vm)

            if vm.selectedProfileSupportsRuntimeSpeedTweak {
                SlewRuntimeTuningView(vm: vm)
            }

            ProfileEditorSectionView(vm: vm)

            Button("Run Continuous Move") {
                Task { await vm.moveContinuous() }
            }
            .buttonStyle(.borderedProminent)
            .disabled(vm.isBusy || capabilities?.motion_active == true)

            if let reason = continuousMoveDisabledReason {
                Text(reason)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            if let error = vm.lastClientError, !error.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
                Text(error)
                    .font(.caption)
                    .foregroundStyle(.red)
            }

            Divider()
            VStack(alignment: .leading, spacing: 4) {
                Text("Control Methods")
                    .font(.headline)
                Text("These methods do different jobs. Output-aware methods use the output encoder and are the real joint-development path. Raw motor-side methods are diagnostic and can disagree with real output motion.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }

            ControlMethodOverviewCardView(showDiagnosticMethods: $showDiagnosticControlMethods)
            BreakawayCharacterizationCardView(vm: vm)
            OutputAwareSpeedControlCardView(vm: vm)
            OutputAwarePositionControlCardView(vm: vm)
            if showDiagnosticControlMethods {
                RawMotorPositionCardView(vm: vm)
                RawMotorSpeedCardView(vm: vm)
            }
            DirectRunQualityCardView(vm: vm)

            MoveDiagnosticsCardView(vm: vm)
        }
        .padding(16)
        .background(Color(nsColor: .windowBackgroundColor), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
    }
}

// MARK: - Control Method Views

private struct ControlMethodCard<Content: View>: View {
    let title: String
    let summary: String
    let badgeText: String?
    let badgeTint: Color
    let content: Content

    init(
        title: String,
        summary: String,
        badgeText: String? = nil,
        badgeTint: Color = .gray,
        @ViewBuilder content: () -> Content
    ) {
        self.title = title
        self.summary = summary
        self.badgeText = badgeText
        self.badgeTint = badgeTint
        self.content = content()
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack(alignment: .top) {
                VStack(alignment: .leading, spacing: 4) {
                    Text(title)
                        .font(.headline)
                    Text(summary)
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
                Spacer()
                if let badgeText {
                    Text(badgeText)
                        .font(.caption.weight(.semibold))
                        .padding(.horizontal, 10)
                        .padding(.vertical, 6)
                        .background(badgeTint.opacity(0.15), in: Capsule())
                }
            }
            content
        }
        .padding(12)
        .background(Color(nsColor: .controlBackgroundColor), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
    }
}

private struct ExplainedInputField: View {
    let title: String
    let detail: String
    @Binding var text: String

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(title)
                .font(.caption.weight(.medium))
                .foregroundStyle(.secondary)
            TextField(title, text: $text)
                .textFieldStyle(.roundedBorder)
            Text(detail)
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
    }
}

private struct ControlMethodStat: View {
    let title: String
    let value: String

    var body: some View {
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
}

private struct ControlMethodOverviewCardView: View {
    @Binding var showDiagnosticMethods: Bool

    var body: some View {
        ControlMethodCard(
            title: "Method Map",
            summary: "Use this as the quick truth table for what each method is actually controlling and how seriously to take it for precision work.",
            badgeText: showDiagnosticMethods ? "diagnostics shown" : "output-aware first",
            badgeTint: showDiagnosticMethods ? .orange : .green
        ) {
            VStack(alignment: .leading, spacing: 8) {
                ControlMethodOverviewRow(
                    title: "Continuous profile move",
                    detail: "Motor/profile-led travel helper. Good for tested travel profiles, not low-speed characterization.",
                    usesOutputEncoder: "No",
                    primaryGoal: "Profile travel",
                    idleBehavior: "Optional",
                    precisionSuitability: "Low"
                )
                ControlMethodOverviewRow(
                    title: "Output-aware speed control",
                    detail: "Uses output encoder. Primary low-speed method. Configurable return to IDLE. Best current path for real joint motion work.",
                    usesOutputEncoder: "Yes",
                    primaryGoal: "Low-speed motion",
                    idleBehavior: "Configurable",
                    precisionSuitability: "Medium"
                )
                ControlMethodOverviewRow(
                    title: "Output-aware position capture",
                    detail: "Uses output encoder. Bounded small-move capture experiment. Always returns to IDLE. Precision-facing, but still experimental.",
                    usesOutputEncoder: "Yes",
                    primaryGoal: "Small capture tests",
                    idleBehavior: "Always",
                    precisionSuitability: "Highest"
                )
                if showDiagnosticMethods {
                    ControlMethodOverviewRow(
                        title: "Raw motor-side position",
                        detail: "Motor encoder only. Diagnostic. Can disagree with real output because the gearbox can absorb or distort the move.",
                        usesOutputEncoder: "No",
                        primaryGoal: "Board diagnostics",
                        idleBehavior: "Optional",
                        precisionSuitability: "Low"
                    )
                    ControlMethodOverviewRow(
                        title: "Raw motor-side speed",
                        detail: "Motor encoder only. Diagnostic. Useful for proof-of-life or A/B checks against output-aware control.",
                        usesOutputEncoder: "No",
                        primaryGoal: "Proof-of-life",
                        idleBehavior: "Optional",
                        precisionSuitability: "Low"
                    )
                }
            }

            Toggle("Show diagnostic raw motor-side methods", isOn: $showDiagnosticMethods)
            Text("When off, the screen stays focused on methods that reason from the output encoder. Turn this on only when you are isolating faults or checking the inner motor-side behavior directly.")
                .font(.caption)
                .foregroundStyle(.secondary)
        }
    }
}

private struct ControlMethodOverviewRow: View {
    let title: String
    let detail: String
    let usesOutputEncoder: String
    let primaryGoal: String
    let idleBehavior: String
    let precisionSuitability: String

    var body: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(title)
                .font(.caption.weight(.semibold))
            Text(detail)
                .font(.caption)
                .foregroundStyle(.secondary)
            HStack(alignment: .top, spacing: 12) {
                ControlMethodStat(title: "Output encoder", value: usesOutputEncoder)
                ControlMethodStat(title: "Primary goal", value: primaryGoal)
                ControlMethodStat(title: "Returns to IDLE", value: idleBehavior)
                ControlMethodStat(title: "Precision fit", value: precisionSuitability)
            }
            .padding(.top, 4)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(10)
        .background(Color(nsColor: .textBackgroundColor), in: RoundedRectangle(cornerRadius: 10, style: .continuous))
    }
}

private struct BreakawayCharacterizationCardView: View {
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

    private func confidenceLabel(_ confidence: VelocitySweepPoint.Confidence) -> String {
        switch confidence {
        case .provisional:
            return "provisional"
        case .repeatable:
            return "repeatable"
        case .unstable:
            return "unstable"
        case .faulted:
            return "faulted"
        }
    }

    private func point(for magnitude: Double, in sweep: VelocitySweepSummary) -> VelocitySweepPoint? {
        sweep.points.first { abs($0.commandTurnsPerSecond - magnitude) <= 1e-6 }
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

    private var bestBandLabel: String {
        guard let sweep,
              let start = sweep.bestBandStartTurnsPerSecond,
              let end = sweep.bestBandEndTurnsPerSecond else {
            return "not proven"
        }
        if abs(start - end) <= 1e-6 {
            return String(format: "%.2f t/s", start)
        }
        return String(format: "%.2f .. %.2f t/s", start, end)
    }

    private var sweepConfidenceLabel: String {
        guard let sweep else { return "not proven" }
        if let usable = sweep.usableFloorTurnsPerSecond, let point = point(for: usable, in: sweep) {
            return confidenceLabel(point.confidence)
        }
        if let breakaway = sweep.breakawayFloorTurnsPerSecond, let point = point(for: breakaway, in: sweep) {
            return confidenceLabel(point.confidence)
        }
        return "not proven"
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
        ControlMethodCard(
            title: "Characterize Breakaway",
            summary: "This is not a production move method. It runs repeated timed motor-side speed probes and scores whether the output encoder saw real breakaway, useful follow, and a repeatable band.",
            badgeText: vm.currentVelocityHint.headline,
            badgeTint: tint(for: vm.currentVelocityHint.tier)
        ) {
            Text(vm.currentVelocityHint.detail)
                .font(.caption)
                .foregroundStyle(.secondary)

            if sweepShowsMotorWithoutOutputBreakaway {
                Text("Motor-side motion is being detected, but the output encoder still is not proving real breakaway at some tested points. Treat that as a plant/transmission issue, not a successful low-speed floor.")
                    .font(.caption)
                    .foregroundStyle(.orange)
            }

            HStack(alignment: .top, spacing: 16) {
                ControlMethodStat(title: "Breakaway floor", value: compact(vm.detectedBreakawayFloorTurnsPerSecond, suffix: "t/s"))
                ControlMethodStat(title: "Usable floor", value: compact(vm.detectedUsableFloorTurnsPerSecond, suffix: "t/s"))
                ControlMethodStat(title: "Best clean band", value: bestBandLabel)
                ControlMethodStat(title: "Sweep confidence", value: sweepConfidenceLabel)
                ControlMethodStat(
                    title: "Assist floor",
                    value: vm.assistFloorTurnsPerSecond.map { String(format: "%.2f t/s (%@)", $0, vm.assistFloorSourceLabel ?? "derived") } ?? "run sweep first"
                )
            }

            HStack(alignment: .top, spacing: 12) {
                ExplainedInputField(
                    title: "Start t/s",
                    detail: "First motor-side speed setpoint tested in the sweep.",
                    text: $vm.directControlForm.sweepStartTurnsPerSecond
                )
                ExplainedInputField(
                    title: "Stop t/s",
                    detail: "Last motor-side speed setpoint tested in the sweep.",
                    text: $vm.directControlForm.sweepStopTurnsPerSecond
                )
                ExplainedInputField(
                    title: "Step t/s",
                    detail: "Spacing between tested motor-side speeds.",
                    text: $vm.directControlForm.sweepStepTurnsPerSecond
                )
                ExplainedInputField(
                    title: "Duration (s)",
                    detail: "How long each probe runs before it is scored.",
                    text: $vm.directControlForm.sweepDurationSeconds
                )
                ExplainedInputField(
                    title: "Trials / point",
                    detail: "Repeat count used before a floor or clean band is promoted.",
                    text: $vm.directControlForm.sweepTrialsPerPoint
                )
            }

            if let sweep {
                Text("\(sweep.trialsPerPoint) trial(s) per point with \(sweep.consensusRequired)/\(sweep.trialsPerPoint) consensus required for promotion.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
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
                                Text("\(point.cleanTrials)/\(point.trialsRun) clean • \(point.followTrials)/\(point.trialsRun) follow • \(confidenceLabel(point.confidence))")
                                    .font(.system(.caption, design: .monospaced))
                                    .foregroundStyle(.secondary)
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
    }
}

private struct OutputAwareSpeedControlCardView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    private var gearRatio: Double? {
        let raw = vm.moveForm.gearRatio.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let value = Double(raw), value > 0 else { return nil }
        return value
    }

    private var requestedMotorSideSpeedTurnsPerSecond: Double? {
        let raw = vm.directControlForm.outputAwareSpeedTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let value = Double(raw), abs(value) > 1e-6 else { return nil }
        return value
    }

    private var equivalentOutputTurnsPerSecond: Double? {
        guard let requestedMotorSideSpeedTurnsPerSecond, let gearRatio else { return nil }
        return requestedMotorSideSpeedTurnsPerSecond / gearRatio
    }

    private var equivalentOutputDegreesPerSecond: Double? {
        equivalentOutputTurnsPerSecond.map { $0 * 360.0 }
    }

    private var assistFloorLabel: String {
        vm.assistFloorTurnsPerSecond.map { String(format: "%.2f t/s (%@)", $0, vm.assistFloorSourceLabel ?? "derived") } ?? "run sweep first"
    }

    var body: some View {
        ControlMethodCard(
            title: "Output-Aware Speed Control",
            summary: "Primary low-speed development method. You command a motor-side speed setpoint, but real breakaway and follow are judged from the output encoder. The controller can kick through breakaway and then trim back to a bounded speed.",
            badgeText: vm.currentVelocityHint.headline,
            badgeTint: {
                switch vm.currentVelocityHint.tier {
                case .usable:
                    return .green
                case .borderline:
                    return .orange
                case .belowFloor:
                    return .red
                case .unknown:
                    return .gray
                }
            }()
        ) {
            HStack(alignment: .top, spacing: 12) {
                ExplainedInputField(
                    title: "Motor-side speed setpoint (turns/s)",
                    detail: "Requested inner-loop motor speed. Ideal output speed is this value divided by gear ratio.",
                    text: $vm.directControlForm.outputAwareSpeedTurnsPerSecond
                )
                ExplainedInputField(
                    title: "Command duration (s)",
                    detail: "Maximum time the speed method is allowed to run before it ends or returns to IDLE.",
                    text: $vm.directControlForm.outputAwareSpeedDurationSeconds
                )
            }
            Toggle("Return to IDLE after timed command", isOn: $vm.directControlForm.outputAwareSpeedReleaseAfter)
            Text("If off, the axis can stay armed after the timed run. For low-speed diagnostics on this plant, returning to IDLE is the safer default.")
                .font(.caption)
                .foregroundStyle(.secondary)

            if let requestedMotorSideSpeedTurnsPerSecond {
                let motorSideDegreesPerSecond = requestedMotorSideSpeedTurnsPerSecond * 360.0
                if let equivalentOutputTurnsPerSecond, let equivalentOutputDegreesPerSecond {
                    Text(String(format: "Setpoint meaning: %.3f motor turns/s = %.1f motor deg/s, ideally %.4f output turns/s (%.1f output deg/s) at %.1f:1 gearing. Real output can be lower if the motor or transmission does not follow.", requestedMotorSideSpeedTurnsPerSecond, motorSideDegreesPerSecond, equivalentOutputTurnsPerSecond, equivalentOutputDegreesPerSecond, gearRatio ?? 25.0))
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }

            HStack(alignment: .top, spacing: 12) {
                ExplainedInputField(
                    title: "Manual floor t/s",
                    detail: "Override the measured floor used for kick-to-breakaway. Leave blank to use sweep evidence.",
                    text: $vm.directControlForm.assistManualFloorTurnsPerSecond
                )
                ExplainedInputField(
                    title: "Kick max (s)",
                    detail: "Longest time the controller may stay above the requested setpoint to force real output breakaway.",
                    text: $vm.directControlForm.assistKickMaxDurationSeconds
                )
                ExplainedInputField(
                    title: "Breakaway out t",
                    detail: "Minimum measured output travel that counts as real breakaway on the output sensor.",
                    text: $vm.directControlForm.assistBreakawayOutputTurns
                )
            }

            Text("Active floor: \(assistFloorLabel). Primary button uses output-speed feedback after breakaway. Fallback button uses the older kick-only logic for A/B comparison.")
                .font(.caption)
                .foregroundStyle(.secondary)

            HStack(spacing: 12) {
                Button("Run Output-Aware Speed Control") {
                    Task { await vm.commandOutputAwareVelocity() }
                }
                .buttonStyle(.borderedProminent)
                .disabled(vm.assistedVelocityDisabledReason != nil)

                Button("Run Kick-Only Assist") {
                    Task { await vm.commandAssistedVelocity() }
                }
                .buttonStyle(.bordered)
                .disabled(vm.assistedVelocityDisabledReason != nil)
            }

            if let reason = vm.assistedVelocityDisabledReason {
                Text(reason)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }
}

private struct OutputAwarePositionControlCardView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    private var targetOutputTurns: Double? {
        let raw = vm.directControlForm.outputAwarePositionDegrees.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let degrees = Double(raw) else { return nil }
        return degrees / 360.0
    }

    private var assistFloorLabel: String {
        vm.assistFloorTurnsPerSecond.map { String(format: "%.2f t/s (%@)", $0, vm.assistFloorSourceLabel ?? "derived") } ?? "run sweep first"
    }

    var body: some View {
        ControlMethodCard(
            title: "Output-Aware Position Capture",
            summary: "Bounded small-move experiment in output space. The backend still drives the motor with bounded speed commands, but target, settle, and abort logic are judged from the output encoder. This path always returns to IDLE on exit.",
            badgeText: "bounded + auto-idle",
            badgeTint: .blue
        ) {
            HStack(alignment: .top, spacing: 12) {
                ExplainedInputField(
                    title: "Target output angle (deg)",
                    detail: "Real joint/output move target. In relative mode this is added to the current output angle; in absolute mode it is measured from output home.",
                    text: $vm.directControlForm.outputAwarePositionDegrees
                )
                ExplainedInputField(
                    title: "Timeout (s)",
                    detail: "Maximum experiment time before the controller aborts, reports the result, and returns to IDLE.",
                    text: $vm.directControlForm.outputAwarePositionTimeoutSeconds
                )
            }
            Toggle("Relative to current output angle", isOn: $vm.directControlForm.outputAwarePositionRelative)
            Text("If off, the target is absolute output angle from the output-sensor home reference. If on, it is a delta from the current output angle.")
                .font(.caption)
                .foregroundStyle(.secondary)

            HStack(alignment: .top, spacing: 12) {
                ExplainedInputField(
                    title: "Output target tol (turns)",
                    detail: "How close the output angle must get before capture is considered inside the target window.",
                    text: $vm.directControlForm.outputAwarePositionTargetToleranceTurns
                )
                ExplainedInputField(
                    title: "Output settle vel tol (turns/s)",
                    detail: "Output speed below this value counts as settled instead of still coasting past the target.",
                    text: $vm.directControlForm.outputAwarePositionTargetVelToleranceTurnsS
                )
            }

            if let targetOutputTurns {
                Text("Target meaning: \(String(format: "%.2f", targetOutputTurns * 360.0)) output deg = \(String(format: "%.4f", targetOutputTurns)) output turns. This method uses assist floor \(assistFloorLabel), kick max \(vm.directControlForm.assistKickMaxDurationSeconds) s, and breakaway \(vm.directControlForm.assistBreakawayOutputTurns) out t from the speed-control section above.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            } else {
                Text("This method reuses the assist floor, kick max, and breakaway threshold from Output-Aware Speed Control so both output-aware methods stay on the same physical assumptions.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }

            Button("Run Output-Aware Position Capture") {
                Task { await vm.commandOutputAwarePosition() }
            }
            .buttonStyle(.borderedProminent)
            .disabled(vm.outputAwarePositionDisabledReason != nil)

            if let reason = vm.outputAwarePositionDisabledReason {
                Text(reason)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }
}

private struct RawMotorPositionCardView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    private var capabilities: BackendCapabilities? { vm.capabilities }

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
        if Double(vm.directControlForm.rawPositionTurns.trimmingCharacters(in: .whitespacesAndNewlines)) == nil {
            return "Motor-turn target must be numeric."
        }
        if Double(vm.directControlForm.rawPositionTargetToleranceTurns.trimmingCharacters(in: .whitespacesAndNewlines)) == nil {
            return "Motor target tolerance must be numeric."
        }
        if Double(vm.directControlForm.rawPositionTargetVelToleranceTurnsS.trimmingCharacters(in: .whitespacesAndNewlines)) == nil {
            return "Motor settle velocity tolerance must be numeric."
        }
        if vm.directControlForm.rawPositionReleaseAfter,
           vm.directControlForm.rawPositionTimeoutSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            return "Timeout is required when releasing after a raw motor-position move."
        }
        if !vm.directControlForm.rawPositionTimeoutSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty,
           Double(vm.directControlForm.rawPositionTimeoutSeconds.trimmingCharacters(in: .whitespacesAndNewlines)) == nil {
            return "Timeout must be numeric."
        }
        return nil
    }

    var body: some View {
        ControlMethodCard(
            title: "Raw Motor-Side Position",
            summary: "Direct motor-encoder position target. Useful for proving motor-side direction, calibration, and gross travel. It is not an output-precision method, because the gearbox can absorb or distort the resulting output motion.",
            badgeText: "diagnostic",
            badgeTint: .gray
        ) {
            HStack(alignment: .top, spacing: 12) {
                ExplainedInputField(
                    title: "Motor turns",
                    detail: "Motor-shaft target. At 25:1, 25.0 motor turns would ideally equal 1.0 output turn.",
                    text: $vm.directControlForm.rawPositionTurns
                )
                ExplainedInputField(
                    title: "Timeout (s)",
                    detail: "How long the board gets to reach and settle the motor-side target before timing out.",
                    text: $vm.directControlForm.rawPositionTimeoutSeconds
                )
            }
            Toggle("Relative to current motor position", isOn: $vm.directControlForm.rawPositionRelativeTurns)
            Text("If off, the command uses the motor encoder’s absolute turn frame, not the output-sensor frame.")
                .font(.caption)
                .foregroundStyle(.secondary)

            HStack(alignment: .top, spacing: 12) {
                ExplainedInputField(
                    title: "Motor target tol (turns)",
                    detail: "Motor-side settle window. The real output can still miss this by more due to compliance or backlash.",
                    text: $vm.directControlForm.rawPositionTargetToleranceTurns
                )
                ExplainedInputField(
                    title: "Motor settle vel tol (turns/s)",
                    detail: "Motor-side speed below this counts as settled for the board-side command.",
                    text: $vm.directControlForm.rawPositionTargetVelToleranceTurnsS
                )
            }
            Toggle("Return to IDLE after timeout-waited move", isOn: $vm.directControlForm.rawPositionReleaseAfter)
            Text("This is for board-side diagnostics. Use Run Quality below to see whether the output encoder agreed with the motor-side story.")
                .font(.caption)
                .foregroundStyle(.secondary)

            Button("Run Raw Motor Position") {
                Task { await vm.commandDirectPosition() }
            }
            .buttonStyle(.borderedProminent)
            .disabled(disabledReason != nil)

            if let disabledReason {
                Text(disabledReason)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }
}

private struct RawMotorSpeedCardView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    private var capabilities: BackendCapabilities? { vm.capabilities }

    private var gearRatio: Double? {
        let raw = vm.moveForm.gearRatio.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let value = Double(raw), value > 0 else { return nil }
        return value
    }

    private var requestedMotorSideSpeedTurnsPerSecond: Double? {
        let raw = vm.directControlForm.rawSpeedTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let value = Double(raw), abs(value) > 1e-6 else { return nil }
        return value
    }

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
        if Double(vm.directControlForm.rawSpeedTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines)) == nil {
            return "Motor-side speed setpoint must be numeric."
        }
        if !vm.directControlForm.rawSpeedDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty,
           Double(vm.directControlForm.rawSpeedDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines)) == nil {
            return "Duration must be numeric."
        }
        if vm.directControlForm.rawSpeedReleaseAfter &&
           vm.directControlForm.rawSpeedDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            return "Duration is required when returning to IDLE after a raw speed run."
        }
        return nil
    }

    var body: some View {
        ControlMethodCard(
            title: "Raw Motor-Side Speed Setpoint",
            summary: "Timed motor-side speed command with no output feedback. Use this only for proof-of-life, A/B comparisons, or fault isolation against the output-aware methods.",
            badgeText: "diagnostic",
            badgeTint: .gray
        ) {
            HStack(alignment: .top, spacing: 12) {
                ExplainedInputField(
                    title: "Motor-side speed setpoint (turns/s)",
                    detail: "Requested motor speed only. The real output speed can be much lower if the plant does not follow.",
                    text: $vm.directControlForm.rawSpeedTurnsPerSecond
                )
                ExplainedInputField(
                    title: "Command duration (s)",
                    detail: "Optional timed run length. Leave empty only if you intend to stop the axis manually.",
                    text: $vm.directControlForm.rawSpeedDurationSeconds
                )
            }
            Toggle("Return to IDLE after timed command", isOn: $vm.directControlForm.rawSpeedReleaseAfter)
            if let requestedMotorSideSpeedTurnsPerSecond, let gearRatio {
                let idealOutputTurnsPerSecond = requestedMotorSideSpeedTurnsPerSecond / gearRatio
                Text(String(format: "Setpoint meaning: %.3f motor turns/s ideally equals %.4f output turns/s (%.1f output deg/s) at %.1f:1 gearing. No output encoder feedback is used here.", requestedMotorSideSpeedTurnsPerSecond, idealOutputTurnsPerSecond, idealOutputTurnsPerSecond * 360.0, gearRatio))
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }

            Button("Run Raw Motor Speed") {
                Task { await vm.commandDirectVelocity() }
            }
            .buttonStyle(.borderedProminent)
            .disabled(disabledReason != nil)

            if let disabledReason {
                Text(disabledReason)
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }
}

private struct DirectRunQualityCardView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    private var baseline: DirectRunBaseline? { vm.activeDirectRunBaseline }
    private var quality: DirectRunQuality? { vm.latestDirectRunQuality }
    private var currentSnapshot: BackendSnapshot? { vm.snapshot }
    private var currentSensor: BackendOutputSensor? { vm.outputSensor }

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

    private func displayMode(_ mode: String) -> String {
        switch mode {
        case "position":
            return "raw motor position"
        case "velocity":
            return "raw motor speed"
        case "velocity assist":
            return "kick-only assist"
        case "output-aware speed":
            return "output-aware speed"
        case "output-aware position":
            return "output-aware position"
        default:
            return mode
        }
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
                    statRow("Mode", displayMode(baseline.mode))
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
                    statRow("Mode", displayMode(quality.mode))
                    statRow(
                        "Controller",
                        quality.outputAwareFeedbackUsed ? "output-aware" : (quality.assistUsed ? "kick-only" : "raw")
                    )
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
                    statRow("Setpoint->out", formattedFraction(quality.outputFollowFraction))
                    statRow(
                        "Stop",
                        quality.finalOutputVelTurnsS.map { quality.assistUsed && abs($0) > 0.01 ? "coasting" : (abs($0) <= 0.01 ? "settled" : "still moving") } ?? "unknown"
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
                    statRow("Peak cmd speed", formattedTurns(quality.peakCommandedMotorSpeedTurnsS, suffix: "t/s"))
                    statRow("Peak motor-side speed", formattedTurns(quality.peakMotorVelTurnsS, suffix: "t/s"))
                    statRow("Peak output speed", formattedTurns(quality.peakOutputVelTurnsS, suffix: "out t/s"))
                }

                if let reached = quality.motorReachedTarget {
                    Text("Motor-side setpoint travel: \(reached ? "reached" : "not reached")")
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
                        statRow("Peak out err", formattedTurns(quality.peakOutputSpeedErrorTurnsS, suffix: "out t/s"))
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

// MARK: - Profile Summary & Move Diagnostics

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
        guard isStartupSensitiveDirectProfileMoveMode(moveMode) else { return nil }

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

                if let notes = editor?.notes, !notes.isEmpty {
                    Text(notes)
                        .lineLimit(3)
                } else if let notes = detail?.notes, !notes.isEmpty {
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
                        compactStat("Board mode", profileDrivePrimitive(for: editor.moveMode).rawValue)
                        if editor.moveMode == ProfileDriveStrategyChoice.directionalTorqueTravel.rawValue, !editor.commandTorqueNm.isEmpty {
                            compactStat("Torque / Kick", "\(editor.commandTorqueNm) / \(editor.stictionKickNm)")
                            compactStat("Vel abort", editor.velAbortTurnsS.isEmpty ? "n/a" : editor.velAbortTurnsS)
                        } else {
                            compactStat("Pos / Vel", "\(editor.posGain) / \(editor.velGain)")
                            compactStat("Vel limit", editor.velLimit)
                        }
                        if editor.moveMode == ProfileDriveStrategyChoice.directionalTorqueTravel.rawValue {
                            compactStat("Handoff turns", editor.handoffWindowTurns.isEmpty ? "n/a" : editor.handoffWindowTurns)
                        } else if isShapedTravelProfileMoveMode(editor.moveMode), !editor.commandVelTurnsS.isEmpty {
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
                            } else if isDirectProfileMoveMode(editor.moveMode) {
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

                            if isDirectProfileMoveMode(editor.moveMode) {
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

                            if isShapedTravelProfileMoveMode(editor.moveMode) {
                                LazyVGrid(columns: [GridItem(.adaptive(minimum: 110), spacing: 8)], spacing: 8) {
                                    if !editor.commandVelTurnsS.isEmpty { metricRow("Cmd vel t/s", editor.commandVelTurnsS) }
                                    if !editor.commandTorqueNm.isEmpty { metricRow("Travel torque Nm", editor.commandTorqueNm) }
                                    if !editor.kickDurationS.isEmpty { metricRow("Kick duration s", editor.kickDurationS) }
                                    if !editor.handoffWindowTurns.isEmpty { metricRow("Handoff turns", editor.handoffWindowTurns) }
                                    if !editor.commandDt.isEmpty { metricRow("Cmd dt s", editor.commandDt) }
                                    if !editor.velAbortTurnsS.isEmpty { metricRow("Vel abort t/s", editor.velAbortTurnsS) }
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
                            if let torque = number("command_torque_nm") {
                                metricRow("Travel torque", String(format: "%.3f Nm", torque))
                            }
                            if let torque = number("kick_torque_nm") {
                                metricRow("Kick torque", String(format: "%.3f Nm", torque))
                            }
                            if let kickDuration = number("kick_duration_s_actual") {
                                metricRow("Kick duration", String(format: "%.3f s", kickDuration))
                            }
                            if let velAbort = number("vel_abort_turns_s") {
                                metricRow("Vel abort", String(format: "%.3f t/s", velAbort))
                            }
                            if let iq = number("peak_iq_set") {
                                metricRow("Peak Iq set", String(format: "%.3f A", iq))
                            }
                            if let iq = number("peak_iq_meas") {
                                metricRow("Peak Iq meas", String(format: "%.3f A", iq))
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

// MARK: - Slider Follow & Dual Axis

struct SliderFollowSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    private var capabilities: BackendCapabilities? { vm.capabilities }

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

// MARK: - Fact Sheet

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

// MARK: - Navigation

enum AppSection: String, CaseIterable, Identifiable {
    case dashboard = "Dashboard"
    case control = "Control"
    case tuning = "Tuning"
    case diagnostics = "Diagnostics"
    case settings = "Settings"

    var id: String { rawValue }

    var icon: String {
        switch self {
        case .dashboard: return "gauge.with.dots.needle.33percent"
        case .control: return "arrow.triangle.2.circlepath"
        case .tuning: return "slider.horizontal.3"
        case .diagnostics: return "stethoscope"
        case .settings: return "gear"
        }
    }
}

// MARK: - Dashboard

struct DashboardSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                // Quick actions
                HStack(spacing: 10) {
                    Button("Make Ready") { Task { await vm.repairAxisState() } }
                        .buttonStyle(.borderedProminent)
                        .disabled(vm.isBusy || vm.capabilities?.motion_active == true)
                    Button("Idle") { Task { await vm.idle() } }
                        .disabled(vm.isBusy || vm.capabilities?.can_idle == false)
                    Button("Clear Errors") { Task { await vm.clearErrors() } }
                        .disabled(vm.isBusy || vm.capabilities?.can_clear_errors == false)
                    Button("Refresh") { Task { await vm.refreshStatus() } }
                        .disabled(vm.isBusy)
                    Spacer()
                    Button("Full Cal") { Task { await vm.startup() } }
                        .disabled(vm.isBusy || vm.capabilities?.can_startup == false)
                }

                ReadinessGridView(vm: vm)

                if vm.outputSensorPortEnv == nil {
                    OutputSensorLaunchWarningView()
                }
                if vm.outputSensor?.configured == true {
                    OutputSensorSectionView(vm: vm)
                }
            }
            .padding(20)
        }
    }
}

// MARK: - Control

struct ControlSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                MoveSectionView(vm: vm)
                SliderFollowSectionView(vm: vm)
                DualAxisSyncSectionView(vm: vm)
            }
            .padding(20)
        }
    }
}

// MARK: - Tuning

struct TuningSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel
    @AppStorage("FOCUI.showDiagnosticControlMethods") private var showDiagnosticMethods = false

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                // Motor Direction
                GroupBox("Motor Direction") {
                    VStack(alignment: .leading, spacing: 8) {
                        Toggle(
                            "Reverse motor direction",
                            isOn: Binding(
                                get: { vm.motorDirectionSelection < 0 },
                                set: { vm.setMotorDirectionSelection($0 ? -1 : 1) }
                            )
                        )
                        Text("Current: \(vm.currentMotorDirection.map { String(format: "%+d", $0) } ?? "unknown")")
                            .font(.caption).foregroundStyle(.secondary)
                        HStack {
                            Button("Auto Detect") { Task { await vm.autoDetectMotorDirection() } }
                                .disabled(vm.isBusy || vm.capabilities?.motion_active == true || vm.capabilities?.startup_ready == false)
                            Button("Apply") { Task { await vm.applyMotorDirection() } }
                                .disabled(vm.isBusy || vm.capabilities?.motion_active == true || vm.currentMotorDirection == vm.motorDirectionSelection)
                        }
                        if let result = vm.autoDirectionResponse {
                            AutoDirectionResultView(response: result)
                        }
                    }
                }

                // Profile editor
                if vm.selectedProfileSupportsRuntimeSpeedTweak {
                    SlewRuntimeTuningView(vm: vm)
                }
                ProfileEditorSectionView(vm: vm)

                // Control methods
                DisclosureGroup("Breakaway Characterization") {
                    BreakawayCharacterizationCardView(vm: vm)
                }
                DisclosureGroup("Output-Aware Speed Control") {
                    OutputAwareSpeedControlCardView(vm: vm)
                }
                DisclosureGroup("Output-Aware Position Control") {
                    OutputAwarePositionControlCardView(vm: vm)
                }

                DisclosureGroup("Raw Motor Diagnostics") {
                    VStack(alignment: .leading, spacing: 12) {
                        RawMotorPositionCardView(vm: vm)
                        RawMotorSpeedCardView(vm: vm)
                    }
                }

                DirectRunQualityCardView(vm: vm)
            }
            .padding(20)
        }
    }
}

// MARK: - Diagnostics

struct DiagnosticsSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                // Actions
                HStack(spacing: 10) {
                    Button("Diagnose") { Task { await vm.runDiagnose() } }
                        .disabled(vm.isBusy)
                    Button("Motor Fact Sheet") { Task { await vm.runFactSheet() } }
                        .disabled(vm.isBusy)
                    Spacer()
                }

                GuidedBringupSectionView(vm: vm)
                TelemetrySectionView(vm: vm, live: vm.liveMonitor)

                // Diagnosis
                if let diagnosis = vm.diagnosis {
                    GroupBox("Diagnosis") {
                        VStack(alignment: .leading, spacing: 8) {
                            if let text = diagnosis.diagnosis {
                                Text(text)
                            }
                            if let commands = diagnosis.commands, !commands.isEmpty {
                                Text("Next Commands").font(.headline)
                                ForEach(commands, id: \.self) { cmd in
                                    Text(cmd).font(.system(.body, design: .monospaced))
                                }
                            }
                            if let notes = diagnosis.notes, !notes.isEmpty {
                                Text("Notes").font(.headline)
                                ForEach(notes, id: \.self) { note in
                                    Text("- \(note)")
                                }
                            }
                        }
                    }
                }

                // Fact sheet
                if let factSheet = vm.factSheet {
                    VStack(alignment: .leading, spacing: 12) {
                        Text("Motor Fact Sheet").font(.title2.bold())
                        if let measured = factSheet.measured_live, !measured.isEmpty {
                            FactSectionView(title: "Measured Live", rows: measured)
                        }
                        if let configured = factSheet.configured, !configured.isEmpty {
                            FactSectionView(title: "Configured", rows: configured)
                        }
                        if let quickChecks = factSheet.quick_checks, !quickChecks.isEmpty {
                            FactSectionView(title: "Quick Checks", rows: quickChecks)
                        }
                    }
                }

                // Raw backend result
                DisclosureGroup("Backend Result") {
                    VStack(alignment: .leading, spacing: 10) {
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
                }

                MoveDiagnosticsCardView(vm: vm)
            }
            .padding(20)
        }
    }
}

// MARK: - Settings

struct SettingsSectionView: View {
    @ObservedObject var vm: OperatorConsoleViewModel

    var body: some View {
        Form {
            Section("Connection") {
                LabeledInputField(title: "Repo root", text: $vm.repoRoot)
                Stepper("Axis \(vm.axisIndex)", value: $vm.axisIndex, in: 0...1)
                HStack {
                    Text("Board")
                    Spacer()
                    Button("Detect") { Task { await vm.discoverSingleAxisBoards() } }
                        .disabled(vm.isBusy)
                    Button("Apply") { Task { await vm.singleAxisContextChanged() } }
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
            }

            Section("Motor Parameters") {
                LabeledInputField(title: "KV estimate", text: $vm.kvEstimate)
                LabeledInputField(title: "Line-line resistance (ohm)", text: $vm.lineLineROhm)
                LabeledInputField(title: "Clear-errors settle (s)", text: $vm.settleSeconds)
            }

            Section("Telemetry") {
                Picker("Stream rate", selection: $vm.telemetryStreamRate) {
                    ForEach(TelemetryStreamRate.allCases) { rate in
                        Text(rate.rawValue).tag(rate)
                    }
                }
                .onChange(of: vm.telemetryStreamRate) { _, newValue in
                    vm.telemetryStreamRateChanged(newValue)
                }
            }

            Section("Debug") {
                Toggle("Debug backend", isOn: $vm.debugMode)
            }
        }
        .formStyle(.grouped)
        .onChange(of: vm.axisIndex) { _, _ in
            Task { await vm.singleAxisContextChanged() }
        }
    }
}

// MARK: - Content View

struct ContentView: View {
    @StateObject private var vm = OperatorConsoleViewModel()
    @State private var selectedSection: AppSection? = .dashboard

    var body: some View {
        NavigationSplitView {
            List(selection: $selectedSection) {
                ForEach(AppSection.allCases) { section in
                    Label(section.rawValue, systemImage: section.icon)
                        .tag(section)
                }
            }
            .navigationSplitViewColumnWidth(min: 160, ideal: 180, max: 220)
        } detail: {
            VStack(spacing: 0) {
                // Status bar
                statusBar
                Divider()
                // Section content
                switch selectedSection ?? .dashboard {
                case .dashboard:
                    DashboardSectionView(vm: vm)
                case .control:
                    ControlSectionView(vm: vm)
                case .tuning:
                    TuningSectionView(vm: vm)
                case .diagnostics:
                    DiagnosticsSectionView(vm: vm)
                case .settings:
                    SettingsSectionView(vm: vm)
                }
            }
        }
        .frame(minWidth: 900, minHeight: 700)
        .task {
            await vm.ensureInitialRefresh()
            await vm.refreshSyncAxesStatus()
            if !vm.selectedProfileEditorLoaded, !vm.moveForm.profileName.isEmpty {
                await vm.loadProfileEditor(name: vm.moveForm.profileName)
            }
        }
        .task(id: vm.capabilities == nil) {
            await vm.ensureCapabilitiesLoaded()
        }
    }

    private var statusBar: some View {
        HStack(spacing: 12) {
            // Connection indicator
            Circle()
                .fill(vm.capabilities?.startup_ready == true ? Color.green : Color.orange)
                .frame(width: 8, height: 8)
            Text(vm.capabilities?.startup_ready == true ? "Ready" : "Not Ready")
                .font(.subheadline.weight(.medium))

            if let vbus = vm.boardDevice?.vbus_voltage {
                Text(String(format: "%.1fV", vbus))
                    .font(.system(.caption, design: .monospaced))
                    .foregroundStyle(.secondary)
            }

            Spacer()

            if let error = vm.lastClientError {
                Text(error)
                    .font(.caption)
                    .foregroundStyle(.red)
                    .lineLimit(1)
                    .truncationMode(.tail)
            }

            if vm.isBusy {
                HStack(spacing: 6) {
                    ProgressView()
                        .controlSize(.small)
                    Text("Working...")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    Button("Cancel") {
                        vm.cancelCurrentAction()
                    }
                    .buttonStyle(.bordered)
                    .controlSize(.small)
                }
            }
        }
        .padding(.horizontal, 16)
        .padding(.vertical, 8)
        .background(Color(nsColor: .windowBackgroundColor))
    }
}
