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
    let requestedOutputTurns: Double?
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
    let outputAwareFeedbackUsed: Bool
    let assistFloorTurnsPerSecond: Double?
    let assistKickTurnsPerSecond: Double?
    let assistKickDurationS: Double?
    let assistBreakawayDetected: Bool?
    let assistBreakawayDetectedAtS: Double?
    let peakCommandedMotorSpeedTurnsS: Double?
    let peakOutputSpeedErrorTurnsS: Double?
}

struct VelocitySweepPoint {
    enum Tier: String {
        case belowFloor
        case borderline
        case usable
        case faulted
    }

    enum Confidence: String {
        case provisional
        case repeatable
        case unstable
        case faulted
    }

    let commandTurnsPerSecond: Double
    let quality: DirectRunQuality
    let tier: Tier
    let trialsRun: Int
    let consensusRequired: Int
    let cleanTrials: Int
    let followTrials: Int
    let faultTrials: Int
    let confidence: Confidence
}

struct VelocitySweepSummary {
    let points: [VelocitySweepPoint]
    let breakawayFloorTurnsPerSecond: Double?
    let usableFloorTurnsPerSecond: Double?
    let bestBandStartTurnsPerSecond: Double?
    let bestBandEndTurnsPerSecond: Double?
    let trialsPerPoint: Int
    let consensusRequired: Int
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

    @Published var telemetrySamples: [TelemetrySample] = []

    func appendTelemetrySample(from result: BackendResponse, fallbackTc: Double?) {
        guard let snap = result.snapshot,
              let posEst = snap.pos_est,
              let velEst = snap.vel_est,
              let iqMeas = snap.Iq_meas,
              let inputPos = snap.input_pos
        else { return }
        let tc = snap.tc ?? fallbackTc
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
        clearTelemetryHistory()
    }

    private func appendTelemetrySample(_ sample: TelemetrySample) {
        telemetrySamples.append(sample)
        if telemetrySamples.count > Self.maxTelemetrySamples {
            telemetrySamples.removeFirst(telemetrySamples.count - Self.maxTelemetrySamples)
        }
    }
}

enum ProfileDrivePrimitive: String, CaseIterable, Identifiable {
    case positionLed = "Position-led"
    case velocityLed = "Velocity-led"
    case torqueCurrentLed = "Current / torque-led"

    var id: String { rawValue }
}

enum ProfileDriveStrategyChoice: String, CaseIterable, Identifiable {
    case trapStrict = "trap_strict"
    case directPosition = "mks_direct_position"
    case directionalDirect = "mks_directional_direct"
    case directionalSlew = "mks_directional_slew_direct"
    case directionalVelocityTravel = "mks_directional_velocity_travel_direct"
    case velocityPointToPoint = "mks_velocity_point_to_point_direct"
    case directionalTorqueTravel = "mks_directional_torque_travel_direct"

    var id: String { rawValue }

    var label: String {
        switch self {
        case .trapStrict:
            return "Trap strict"
        case .directPosition:
            return "Direct position"
        case .directionalDirect:
            return "Directional direct"
        case .directionalSlew:
            return "Directional slew"
        case .directionalVelocityTravel:
            return "Velocity travel + direct capture"
        case .velocityPointToPoint:
            return "Velocity point-to-point"
        case .directionalTorqueTravel:
            return "Torque travel + direct capture"
        }
    }

    var summary: String {
        switch self {
        case .trapStrict:
            return "Board-side position move using trap trajectory planning. Best when you want conservative motor-space travel and a conventional settle path."
        case .directPosition:
            return "Board-side direct position target with final capture. Minimal travel shaping. Good for proving direct-settle behavior without the directional preload logic."
        case .directionalDirect:
            return "Board-side direct position target with directional preload behavior before final settle. Useful when approach direction matters more than travel smoothness."
        case .directionalSlew:
            return "Board-side position capture with a shaped direct travel law. Good when you need smoother travel than plain directional direct but still want direct final settle."
        case .directionalVelocityTravel:
            return "Velocity-led travel phase with direct final capture. Best when the plant hunts during pure position-led travel and you need a more decisive approach."
        case .velocityPointToPoint:
            return "Pure velocity-led point-to-point travel with direct final capture. Most aggressive saved velocity-profile option in this editor."
        case .directionalTorqueTravel:
            return "Torque-led travel phase with direct final capture near target. Experimental and bounded. Use this when you want the board to push with a travel torque ceiling instead of a travel speed setpoint."
        }
    }

    var keyFields: [String] {
        switch self {
        case .trapStrict:
            return ["Trap vel/acc/dec", "Target tol", "Target vel tol", "Timeout"]
        case .directPosition:
            return ["Run current A", "Final hold s", "Abort abs turns", "Target tol"]
        case .directionalDirect:
            return ["Pre hold s", "Final hold s", "Abort abs turns", "Run current A"]
        case .directionalSlew:
            return ["Cmd vel t/s", "Handoff turns", "Cmd dt s", "Travel gains / limits"]
        case .directionalVelocityTravel:
            return ["Cmd vel t/s", "Handoff turns", "Cmd dt s", "Pre/final hold"]
        case .velocityPointToPoint:
            return ["Cmd vel t/s", "Handoff turns", "Cmd dt s", "Final hold s"]
        case .directionalTorqueTravel:
            return ["Travel torque Nm", "Kick torque Nm", "Kick duration s", "Vel abort t/s"]
        }
    }

    var boardPrimitive: ProfileDrivePrimitive {
        switch self {
        case .directionalVelocityTravel, .velocityPointToPoint:
            return .velocityLed
        case .directionalTorqueTravel:
            return .torqueCurrentLed
        default:
            return .positionLed
        }
    }
}

func normalizedProfileMoveMode(_ raw: String) -> String {
    raw.trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
}

func profileStrategyChoice(for moveMode: String) -> ProfileDriveStrategyChoice {
    switch normalizedProfileMoveMode(moveMode) {
    case ProfileDriveStrategyChoice.directPosition.rawValue:
        return .directPosition
    case ProfileDriveStrategyChoice.directionalDirect.rawValue:
        return .directionalDirect
    case ProfileDriveStrategyChoice.directionalSlew.rawValue:
        return .directionalSlew
    case ProfileDriveStrategyChoice.directionalVelocityTravel.rawValue:
        return .directionalVelocityTravel
    case ProfileDriveStrategyChoice.velocityPointToPoint.rawValue:
        return .velocityPointToPoint
    case ProfileDriveStrategyChoice.directionalTorqueTravel.rawValue:
        return .directionalTorqueTravel
    default:
        return .trapStrict
    }
}

func profileDrivePrimitive(for moveMode: String) -> ProfileDrivePrimitive {
    profileStrategyChoice(for: moveMode).boardPrimitive
}

func isDirectProfileMoveMode(_ moveMode: String) -> Bool {
    switch profileStrategyChoice(for: moveMode) {
    case .trapStrict:
        return false
    case .directPosition, .directionalDirect, .directionalSlew, .directionalVelocityTravel, .velocityPointToPoint, .directionalTorqueTravel:
        return true
    }
}

func isShapedTravelProfileMoveMode(_ moveMode: String) -> Bool {
    switch profileStrategyChoice(for: moveMode) {
    case .directionalSlew, .directionalVelocityTravel, .velocityPointToPoint, .directionalTorqueTravel:
        return true
    case .trapStrict, .directPosition, .directionalDirect:
        return false
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
    @Published var boardState = BackendBoardState()

    private let backend = BackendClient()
    let liveMonitor = LiveMonitorModel()
    private var sliderDebounceTask: Task<Void, Never>?
    private var sliderQueuedAngle: Double?
    private var sliderCommandActive = false
    private var telemetryRequestActive = false
    private var backendEventTask: Task<Void, Never>?
    private var recoveryRefreshTask: Task<Void, Never>?
    private var initialRefreshDone = false
    private var streamingStarted = false
    private var motorDirectionDirty = false
    private var actionTimeoutTask: Task<Void, Never>?

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
    var capabilities: BackendCapabilities? { boardState.capabilities }
    var outputSensor: BackendOutputSensor? { boardState.outputSensor }
    var outerLoop: BackendOuterLoop? { boardState.outputSensor?.outer_loop }
    var boardDevice: BackendDevice? { boardState.device }
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
        let raw = directControlForm.outputAwareSpeedTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let requestedTurnsPerSecond = Double(raw), abs(requestedTurnsPerSecond) > 1e-6 else {
            return DirectVelocityHint(
                tier: .unknown,
                headline: "Need a speed setpoint",
                detail: "Enter a nonzero timed motor-side speed setpoint before judging low-speed quality."
            )
        }
        let magnitude = abs(requestedTurnsPerSecond)
        if let usable = detectedUsableFloorTurnsPerSecond {
            if magnitude >= usable {
                let bestBandDetail: String = {
                    guard let sweep = latestVelocitySweep,
                          let start = sweep.bestBandStartTurnsPerSecond,
                          let end = sweep.bestBandEndTurnsPerSecond
                    else {
                        return String(format: "Command is at or above the tested usable floor of %.2f t/s.", usable)
                    }
                    if abs(start - end) <= 1e-6 {
                        return String(format: "Command is at or above the tested usable point of %.2f t/s.", start)
                    }
                    return String(format: "Command is within or above the tested clean band of %.2f .. %.2f t/s.", start, end)
                }()
                return DirectVelocityHint(
                    tier: .usable,
                    headline: "Usable",
                    detail: bestBandDetail
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
    var diagnosis: BackendDiagnosis? { boardState.diagnosis }
    var factSheet: FactSheet? { boardState.factSheet }
    var snapshot: BackendSnapshot? { boardState.snapshot }
    var profiles: [String] { response?.available_profiles ?? [] }
    var profileDetails: [BackendProfileDetail] { response?.available_profile_details ?? [] }
    private var readinessCapabilities: BackendCapabilities? { boardState.capabilities }
    private var readinessSnapshot: BackendSnapshot? { boardState.snapshot }
    var selectedProfileDetail: BackendProfileDetail? { profileDetails.first(where: { $0.name == moveForm.profileName }) }
    var selectedProfileEditorLoaded: Bool {
        !profileEditor.loadedProfileName.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
    }
    var selectedProfileMoveMode: String {
        selectedProfileEditorLoaded
            ? profileEditor.moveMode.trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
            : selectedProfileDetail?.move_mode?.trimmingCharacters(in: .whitespacesAndNewlines).lowercased() ?? ""
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
        if velocitySweepTrialsPerPoint() == nil {
            return "Trials per point must be an integer from 1 to 9."
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
        if Double(directControlForm.outputAwareSpeedTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines)) == nil {
            return "Motor-side speed setpoint must be numeric."
        }
        if directControlForm.outputAwareSpeedDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            return "Duration is required for assisted velocity."
        }
        if Double(directControlForm.outputAwareSpeedDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines)) == nil {
            return "Duration must be numeric."
        }
        if assistFloorTurnsPerSecond == nil {
            return "Run sweep first or set a manual floor override."
        }
        return nil
    }

    var outputAwarePositionDisabledReason: String? {
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
            return "Healthy output-sensor telemetry is required for output-aware position."
        }
        guard let outputDegrees = Double(directControlForm.outputAwarePositionDegrees.trimmingCharacters(in: .whitespacesAndNewlines)) else {
            return "Target output angle must be numeric."
        }
        if directControlForm.outputAwarePositionRelative && abs(outputDegrees) <= 1e-6 {
            return "Relative output-angle target must be nonzero."
        }
        guard let timeout = Double(directControlForm.outputAwarePositionTimeoutSeconds.trimmingCharacters(in: .whitespacesAndNewlines)),
              timeout > 0 else {
            return "Timeout must be a positive number."
        }
        guard Double(directControlForm.outputAwarePositionTargetToleranceTurns.trimmingCharacters(in: .whitespacesAndNewlines)) != nil else {
            return "Output target tolerance must be numeric."
        }
        guard Double(directControlForm.outputAwarePositionTargetVelToleranceTurnsS.trimmingCharacters(in: .whitespacesAndNewlines)) != nil else {
            return "Output settle velocity tolerance must be numeric."
        }
        if assistFloorTurnsPerSecond == nil {
            return "Run sweep first or set a manual floor override so output-aware position has a breakaway floor."
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

    var selectedProfileBrowsePrimitive: ProfileDrivePrimitive {
        if let primitive = ProfileDrivePrimitive(rawValue: moveForm.profileBrowsePrimitive) {
            return primitive
        }
        return .positionLed
    }

    func drivePrimitive(for detail: BackendProfileDetail) -> ProfileDrivePrimitive {
        if let raw = detail.board_primitive?.trimmingCharacters(in: .whitespacesAndNewlines),
           let primitive = ProfileDrivePrimitive(rawValue: raw) {
            return primitive
        }
        if let moveMode = detail.move_mode {
            return profileDrivePrimitive(for: moveMode)
        }
        return .positionLed
    }

    func isBuiltInProfile(_ detail: BackendProfileDetail) -> Bool {
        let source = (detail.source ?? "").trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
        return source.contains("builtin")
    }

    var filteredTemplateProfiles: [BackendProfileDetail] {
        profileDetails.filter { detail in
            isBuiltInProfile(detail) && drivePrimitive(for: detail) == selectedProfileBrowsePrimitive
        }
    }

    var filteredManualProfiles: [BackendProfileDetail] {
        profileDetails.filter { detail in
            !isBuiltInProfile(detail) && drivePrimitive(for: detail) == selectedProfileBrowsePrimitive
        }
    }

    private func syncProfileBrowserStateFromActiveProfile() {
        if ProfileDrivePrimitive(rawValue: moveForm.profileBrowsePrimitive) == nil {
            moveForm.profileBrowsePrimitive = ProfileDrivePrimitive.positionLed.rawValue
        }
        if let selectedProfileDetail,
           drivePrimitive(for: selectedProfileDetail) == selectedProfileBrowsePrimitive {
            if isBuiltInProfile(selectedProfileDetail) {
                moveForm.templateProfileSelection = selectedProfileDetail.name
                if moveForm.manualProfileSelection == selectedProfileDetail.name {
                    moveForm.manualProfileSelection = ""
                }
            } else {
                moveForm.manualProfileSelection = selectedProfileDetail.name
                if moveForm.templateProfileSelection == selectedProfileDetail.name {
                    moveForm.templateProfileSelection = ""
                }
            }
        }
        if moveForm.templateProfileSelection.isEmpty, let first = filteredTemplateProfiles.first {
            moveForm.templateProfileSelection = first.name
        }
        if !moveForm.templateProfileSelection.isEmpty,
           !filteredTemplateProfiles.contains(where: { $0.name == moveForm.templateProfileSelection }) {
            moveForm.templateProfileSelection = filteredTemplateProfiles.first?.name ?? ""
        }
        if !moveForm.manualProfileSelection.isEmpty,
           !filteredManualProfiles.contains(where: { $0.name == moveForm.manualProfileSelection }) {
            moveForm.manualProfileSelection = filteredManualProfiles.first?.name ?? ""
        }
    }

    func setProfileBrowsePrimitive(_ primitive: ProfileDrivePrimitive) {
        moveForm.profileBrowsePrimitive = primitive.rawValue
        if !filteredTemplateProfiles.contains(where: { $0.name == moveForm.templateProfileSelection }) {
            moveForm.templateProfileSelection = filteredTemplateProfiles.first?.name ?? ""
        }
        if !filteredManualProfiles.contains(where: { $0.name == moveForm.manualProfileSelection }) {
            moveForm.manualProfileSelection = filteredManualProfiles.first?.name ?? ""
        }
    }

    func loadSelectedTemplateProfile() async {
        let name = moveForm.templateProfileSelection.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !name.isEmpty else { return }
        await loadProfileEditor(name: name)
    }

    func loadSelectedManualProfile() async {
        let name = moveForm.manualProfileSelection.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !name.isEmpty else { return }
        await loadProfileEditor(name: name)
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

    func cancelCurrentAction() {
        actionTimeoutTask?.cancel()
        actionTimeoutTask = nil
        blockingActionInFlight = false
        lastClientError = "Action cancelled."
    }

    func repairAxisState() async {
        blockingActionInFlight = true
        lastClientError = nil
        let timeoutTask = Task { @MainActor [weak self] in
            try? await Task.sleep(nanoseconds: 120_000_000_000)
            guard let self, self.blockingActionInFlight else { return }
            self.blockingActionInFlight = false
            self.lastClientError = "Backend action timed out. Use Refresh Status to check board state."
        }
        actionTimeoutTask = timeoutTask
        defer {
            blockingActionInFlight = false
            actionTimeoutTask?.cancel()
            actionTimeoutTask = nil
        }
        do {
            let startupProfile = preferredMksStartupProfileName.trimmingCharacters(in: .whitespacesAndNewlines)
            var arguments = ["--timeout-s", "30", "--repair-passes", "5"]
            if !startupProfile.isEmpty {
                arguments += ["--profile-name", startupProfile, "--mks-fallback-profile-name", startupProfile]
            } else if !selectedProfileName.isEmpty {
                arguments += ["--profile-name", selectedProfileName]
            }
            let result = try await backend.run(action: "make-ready", arguments: arguments, context: requestContext())
            response = result
            mergeProfilesIfNeeded(from: result)
            mergeProfileEditorIfNeeded(from: result)
            mergeBoardStateIfNeeded(from: result)
            liveMonitor.appendTelemetrySample(from: result, fallbackTc: snapshot?.tc)

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
            handleBackendError(error)
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

    // ── Outer-loop (RP2040 PD cascade) ──────────────────────────────────────
    func outerLoopEnable() async { await run(action: "outer-loop-enable", countsAsBlocking: false) }
    func outerLoopDisable() async { await run(action: "outer-loop-disable", countsAsBlocking: false) }
    func outerLoopSetSetpoint(_ outputTurns: Double) async {
        await run(action: "outer-loop-setpoint",
                  arguments: ["--output-turns", String(format: "%.6f", outputTurns)],
                  countsAsBlocking: false)
    }
    func outerLoopSetGains(kp: Double, kd: Double, velLimit: Double) async {
        await run(action: "outer-loop-gains",
                  arguments: ["--outer-loop-kp", String(format: "%.4f", kp),
                              "--outer-loop-kd", String(format: "%.4f", kd),
                              "--outer-loop-vel-limit", String(format: "%.4f", velLimit)],
                  countsAsBlocking: false)
    }
    func outerLoopRefreshStatus() async { await run(action: "outer-loop-status", countsAsBlocking: false) }

    func setMotorDirectionSelection(_ direction: Int) {
        motorDirectionSelection = (direction < 0 ? -1 : 1)
        motorDirectionDirty = true
    }

    func autoDetectMotorDirection() async {
        blockingActionInFlight = true
        lastClientError = nil
        let timeoutTask = Task { @MainActor [weak self] in
            try? await Task.sleep(nanoseconds: 120_000_000_000)
            guard let self, self.blockingActionInFlight else { return }
            self.blockingActionInFlight = false
            self.lastClientError = "Backend action timed out. Use Refresh Status to check board state."
        }
        actionTimeoutTask = timeoutTask
        defer {
            blockingActionInFlight = false
            actionTimeoutTask?.cancel()
            actionTimeoutTask = nil
        }
        do {
            let result = try await backend.run(action: "auto-direction-contract", arguments: [], context: requestContext())
            response = result
            autoDirectionResponse = result
            mergeProfilesIfNeeded(from: result)
            mergeProfileEditorIfNeeded(from: result)
            mergeBoardStateIfNeeded(from: result)
        } catch {
            handleBackendError(error)
        }
        motorDirectionDirty = false
    }

    func runGuidedBringup() async {
        blockingActionInFlight = true
        lastClientError = nil
        let timeoutTask = Task { @MainActor [weak self] in
            try? await Task.sleep(nanoseconds: 120_000_000_000)
            guard let self, self.blockingActionInFlight else { return }
            self.blockingActionInFlight = false
            self.lastClientError = "Backend action timed out. Use Refresh Status to check board state."
        }
        actionTimeoutTask = timeoutTask
        defer {
            blockingActionInFlight = false
            actionTimeoutTask?.cancel()
            actionTimeoutTask = nil
        }
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
            mergeBoardStateIfNeeded(from: result)
        } catch {
            handleBackendError(error)
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

    private func velocitySweepTrialsPerPoint() -> Int? {
        let raw = directControlForm.sweepTrialsPerPoint.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let trials = Int(raw), trials > 0, trials <= 9 else { return nil }
        return trials
    }

    private func sweepConsensusRequired(for trials: Int) -> Int {
        max(1, Int(ceil(Double(max(1, trials)) * (2.0 / 3.0))))
    }

    private func median(_ values: [Double]) -> Double? {
        guard !values.isEmpty else { return nil }
        let sorted = values.sorted()
        let mid = sorted.count / 2
        if sorted.count.isMultiple(of: 2) {
            return (sorted[mid - 1] + sorted[mid]) / 2.0
        }
        return sorted[mid]
    }

    private func median(_ values: [Double?]) -> Double? {
        median(values.compactMap { $0 })
    }

    private func consensusBool(_ values: [Bool?], threshold: Int) -> Bool? {
        let trueCount = values.compactMap { $0 }.filter { $0 }.count
        let falseCount = values.compactMap { $0 }.filter { !$0 }.count
        if trueCount >= threshold {
            return true
        }
        if falseCount >= threshold {
            return false
        }
        return nil
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

    private func aggregateVelocitySweepPoint(commandTurnsPerSecond: Double, trialQualities: [DirectRunQuality]) -> VelocitySweepPoint {
        let trialsRun = trialQualities.count
        let consensusRequired = sweepConsensusRequired(for: trialsRun)
        let cleanTrials = trialQualities.filter { $0.verdict == .good }.count
        let followTrials = trialQualities.filter { quality in
            quality.verdict == .good || quality.verdict == .partial || quality.verdict == .signInverted
        }.count
        let faultTrials = trialQualities.filter { quality in
            quality.verdict == .sensorMismatch || quality.verdict == .wrongDirection || quality.verdict == .faulted
        }.count
        let weakTrials = trialQualities.filter { quality in
            quality.verdict == .stalled || quality.verdict == .informational
        }.count

        let tier: VelocitySweepPoint.Tier
        if faultTrials >= consensusRequired {
            tier = .faulted
        } else if cleanTrials >= consensusRequired {
            tier = .usable
        } else if followTrials >= consensusRequired {
            tier = .borderline
        } else {
            tier = .belowFloor
        }

        let confidence: VelocitySweepPoint.Confidence
        if faultTrials >= consensusRequired {
            confidence = .faulted
        } else if trialsRun < 3 {
            confidence = .provisional
        } else if cleanTrials >= consensusRequired || followTrials >= consensusRequired || weakTrials >= consensusRequired {
            confidence = .repeatable
        } else {
            confidence = .unstable
        }

        let verdict: DirectRunQuality.Verdict
        let headline: String
        let explanation: String
        switch tier {
        case .usable:
            verdict = .good
            headline = (confidence == .provisional) ? "Provisionally clean" : "Clean enough"
            explanation = "\(cleanTrials)/\(trialsRun) trials were clean enough. Median metrics are shown."
        case .borderline:
            verdict = .partial
            headline = (confidence == .unstable) ? "Mixed follow" : "Partial follow"
            explanation = "\(followTrials)/\(trialsRun) trials showed real output follow, but the median result stayed below a clean-enough pass."
        case .belowFloor:
            verdict = .stalled
            headline = (confidence == .unstable) ? "Mixed / weak" : "Weak or stalled"
            explanation = "\(weakTrials)/\(trialsRun) trials stayed below useful output motion."
        case .faulted:
            verdict = .faulted
            headline = "Faulted"
            explanation = "\(faultTrials)/\(trialsRun) trials faulted or produced invalid output evidence."
        }

        let firstMode = trialQualities.first?.mode ?? "velocity"
        let representativeError = trialQualities.compactMap(\.errorSummary).first
        let quality = DirectRunQuality(
            mode: firstMode,
            verdict: verdict,
            headline: headline,
            explanation: explanation,
            expectedMotorDeltaTurns: median(trialQualities.map(\.expectedMotorDeltaTurns)),
            actualMotorDeltaTurns: median(trialQualities.map(\.actualMotorDeltaTurns)),
            furthestMotorDeltaTurns: median(trialQualities.map(\.furthestMotorDeltaTurns)),
            maxAbsMotorDeltaTurns: median(trialQualities.map(\.maxAbsMotorDeltaTurns)),
            motorFollowFraction: median(trialQualities.map(\.motorFollowFraction)),
            expectedOutputDeltaTurns: median(trialQualities.map(\.expectedOutputDeltaTurns)),
            expectedOutputFromActualMotorTurns: median(trialQualities.map(\.expectedOutputFromActualMotorTurns)),
            actualOutputDeltaTurns: median(trialQualities.map(\.actualOutputDeltaTurns)),
            furthestOutputDeltaTurns: median(trialQualities.map(\.furthestOutputDeltaTurns)),
            maxAbsOutputDeltaTurns: median(trialQualities.map(\.maxAbsOutputDeltaTurns)),
            outputFollowFraction: median(trialQualities.map(\.outputFollowFraction)),
            transmissionFollowFraction: median(trialQualities.map(\.transmissionFollowFraction)),
            directionCorrect: consensusBool(trialQualities.map(\.directionCorrect), threshold: consensusRequired),
            motorReachedTarget: consensusBool(trialQualities.map(\.motorReachedTarget), threshold: consensusRequired),
            finalOutputVelTurnsS: median(trialQualities.map(\.finalOutputVelTurnsS)),
            lagDeltaOutputTurns: median(trialQualities.map(\.lagDeltaOutputTurns)),
            peakMotorVelTurnsS: median(trialQualities.map(\.peakMotorVelTurnsS)),
            peakOutputVelTurnsS: median(trialQualities.map(\.peakOutputVelTurnsS)),
            errorSummary: representativeError,
            assistUsed: false,
            outputAwareFeedbackUsed: false,
            assistFloorTurnsPerSecond: nil,
            assistKickTurnsPerSecond: nil,
            assistKickDurationS: nil,
            assistBreakawayDetected: nil,
            assistBreakawayDetectedAtS: nil,
            peakCommandedMotorSpeedTurnsS: nil,
            peakOutputSpeedErrorTurnsS: nil
        )

        return VelocitySweepPoint(
            commandTurnsPerSecond: commandTurnsPerSecond,
            quality: quality,
            tier: tier,
            trialsRun: trialsRun,
            consensusRequired: consensusRequired,
            cleanTrials: cleanTrials,
            followTrials: followTrials,
            faultTrials: faultTrials,
            confidence: confidence
        )
    }

    private func summarizeVelocitySweep(points: [VelocitySweepPoint], stoppedReason: String?, trialsPerPoint: Int) -> VelocitySweepSummary {
        let breakaway = points
            .filter { $0.followTrials >= $0.consensusRequired }
            .map { abs($0.commandTurnsPerSecond) }
            .min()
        let usable = points
            .filter { $0.cleanTrials >= $0.consensusRequired }
            .map { abs($0.commandTurnsPerSecond) }
            .min()
        var bestRunStartIndex: Int?
        var bestRunEndIndex: Int?
        var currentStartIndex: Int?
        for (index, point) in points.enumerated() {
            if point.tier == .usable {
                if currentStartIndex == nil {
                    currentStartIndex = index
                }
            } else {
                currentStartIndex = nil
            }
            guard let runStart = currentStartIndex else { continue }
            let currentLength = index - runStart + 1
            let bestLength = {
                guard let bestRunStartIndex, let bestRunEndIndex else { return 0 }
                return bestRunEndIndex - bestRunStartIndex + 1
            }()
            if currentLength > bestLength {
                bestRunStartIndex = runStart
                bestRunEndIndex = index
            }
        }
        let bestBandStart: Double? = {
            guard let start = bestRunStartIndex, let end = bestRunEndIndex else { return nil }
            return min(abs(points[start].commandTurnsPerSecond), abs(points[end].commandTurnsPerSecond))
        }()
        let bestBandEnd: Double? = {
            guard let start = bestRunStartIndex, let end = bestRunEndIndex else { return nil }
            return max(abs(points[start].commandTurnsPerSecond), abs(points[end].commandTurnsPerSecond))
        }()
        let consensusRequired = sweepConsensusRequired(for: trialsPerPoint)
        return VelocitySweepSummary(
            points: points,
            breakawayFloorTurnsPerSecond: breakaway,
            usableFloorTurnsPerSecond: usable,
            bestBandStartTurnsPerSecond: bestBandStart,
            bestBandEndTurnsPerSecond: bestBandEnd,
            trialsPerPoint: trialsPerPoint,
            consensusRequired: consensusRequired,
            stoppedReason: stoppedReason,
            completedAt: Date()
        )
    }

    private func captureDirectRunBaseline(
        mode: String,
        requestedMotorTurns: Double? = nil,
        requestedOutputTurns: Double? = nil,
        requestedTurnsPerSecond: Double? = nil,
        requestedDurationS: Double? = nil,
        relative: Bool = false,
        releaseAfter: Bool = false
    ) -> DirectRunBaseline {
        let snap = snapshot
        let sensor = outputSensor
        return DirectRunBaseline(
            mode: mode,
            startedAt: Date(),
            startMotorTurns: snap?.pos_est,
            startOutputTurns: sensor?.output_turns,
            startOutputVelTurnsS: sensor?.output_vel_turns_s,
            startLagOutputTurns: sensor?.compliance_lag_output_turns,
            requestedMotorTurns: requestedMotorTurns,
            requestedOutputTurns: requestedOutputTurns,
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

        let expectedOutputDeltaFromOutputAwarePosition: Double? = {
            guard baseline.mode == "output-aware position" else { return nil }
            if let start = resultObject["start_output_turns"]?.numberValue ?? baseline.startOutputTurns,
               let target = resultObject["target_output_turns"]?.numberValue {
                return target - start
            }
            if let requestedOutputTurns = resultObject["requested_output_turns"]?.numberValue ?? baseline.requestedOutputTurns {
                if baseline.relative {
                    return requestedOutputTurns
                }
                if let start = baseline.startOutputTurns {
                    return requestedOutputTurns - start
                }
                return requestedOutputTurns
            }
            return nil
        }()

        let expectedMotorDelta: Double? = {
            if baseline.mode == "position" {
                if let start = resultObject["start_pos_turns"]?.numberValue,
                   let target = resultObject["target_turns"]?.numberValue {
                    return target - start
                }
                return resultObject["requested_turns"]?.numberValue ?? baseline.requestedMotorTurns
            }
            if baseline.mode == "output-aware position" {
                if let expectedOutputDeltaFromOutputAwarePosition {
                    return expectedOutputDeltaFromOutputAwarePosition * gearRatio
                }
                return baseline.requestedMotorTurns
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

        let liveOrResultSensor = result.output_sensor ?? outputSensor
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
        let expectedOutputDelta: Double? = {
            if baseline.mode == "output-aware position" {
                return expectedOutputDeltaFromOutputAwarePosition
            }
            return expectedMotorDelta.map { $0 / gearRatio }
        }()
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
            outputAwareFeedbackUsed: resultObject["output_aware_feedback_used"]?.boolValue ?? false,
            assistFloorTurnsPerSecond: resultObject["assist_floor_turns_s"]?.numberValue,
            assistKickTurnsPerSecond: resultObject["kick_turns_per_second"]?.numberValue,
            assistKickDurationS: resultObject["kick_duration_s_actual"]?.numberValue,
            assistBreakawayDetected: resultObject["breakaway_detected"]?.boolValue,
            assistBreakawayDetectedAtS: resultObject["breakaway_detected_at_s"]?.numberValue,
            peakCommandedMotorSpeedTurnsS: resultObject["peak_commanded_motor_speed_turns_s"]?.numberValue,
            peakOutputSpeedErrorTurnsS: resultObject["peak_output_speed_error_turns_s"]?.numberValue
        )
    }

    func pollTelemetryOnce() async {
        guard !telemetryRequestActive else { return }
        telemetryRequestActive = true
        defer { telemetryRequestActive = false }
        do {
            let result = try await backend.run(action: "telemetry", arguments: [], context: requestContext())
            mergeProfilesIfNeeded(from: result)
            mergeBoardStateIfNeeded(from: result)
            liveMonitor.appendTelemetrySample(from: result, fallbackTc: snapshot?.tc)
        } catch {
            handleBackendError(error)
            telemetryAutoRefresh = false
        }
    }

    func moveContinuous() async {
        await ensureStreaming()
        if !selectedProfileEditorLoaded && !moveForm.profileName.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            await loadProfileEditor(name: moveForm.profileName)
        }
        guard selectedProfileEditorLoaded else {
            lastClientError = "Load a profile into the editor before running a continuous move."
            return
        }
        let profilePayload: String
        do {
            profilePayload = try profileEditor.jsonPayload()
        } catch {
            handleBackendError(error)
            return
        }
        let effectiveProfileName = profileEditor.name.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
            ? moveForm.profileName
            : profileEditor.name
        var args = [
            "--angle-deg", moveForm.angleDeg,
            "--angle-space", moveForm.angleSpace,
            "--profile-name", effectiveProfileName,
            "--profile-json", profilePayload,
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
        let requestedTurns = Double(directControlForm.rawPositionTurns.trimmingCharacters(in: .whitespacesAndNewlines))
        let baseline = captureDirectRunBaseline(
            mode: "position",
            requestedMotorTurns: requestedTurns,
            relative: directControlForm.rawPositionRelativeTurns,
            releaseAfter: directControlForm.rawPositionReleaseAfter
        )
        activeDirectRunBaseline = baseline
        var args = [
            "--turns", directControlForm.rawPositionTurns,
            "--target-tolerance-turns", directControlForm.rawPositionTargetToleranceTurns,
            "--target-vel-tolerance-turns-s", directControlForm.rawPositionTargetVelToleranceTurnsS,
            "--gear-ratio", moveForm.gearRatio,
        ]
        if directControlForm.rawPositionRelativeTurns {
            args.append("--relative")
        }
        if !directControlForm.rawPositionTimeoutSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty {
            args.append(contentsOf: ["--timeout-s", directControlForm.rawPositionTimeoutSeconds])
        }
        if directControlForm.rawPositionReleaseAfter {
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
        let hasTimedDuration = !directControlForm.rawSpeedDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
        let requestedTurnsPerSecond = Double(directControlForm.rawSpeedTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines))
        let requestedDuration = hasTimedDuration
            ? Double(directControlForm.rawSpeedDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines))
            : nil
        let baseline = captureDirectRunBaseline(
            mode: "velocity",
            requestedTurnsPerSecond: requestedTurnsPerSecond,
            requestedDurationS: requestedDuration,
            releaseAfter: directControlForm.rawSpeedReleaseAfter
        )
        activeDirectRunBaseline = baseline
        var args = [
            "--turns-per-second", directControlForm.rawSpeedTurnsPerSecond,
            "--gear-ratio", moveForm.gearRatio,
        ]
        if hasTimedDuration {
            args.append(contentsOf: ["--duration-s", directControlForm.rawSpeedDurationSeconds])
        }
        if directControlForm.rawSpeedReleaseAfter {
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
              let trialsPerPoint = velocitySweepTrialsPerPoint(),
              duration > 0.0
        else {
            lastClientError = "Sweep start/stop/step/duration/trials must be valid numbers."
            return
        }
        let startedTemporaryStream = !streamingStarted
        if startedTemporaryStream {
            await ensureStreaming()
        }
        blockingActionInFlight = true
        let timeoutTask = Task { @MainActor [weak self] in
            try? await Task.sleep(nanoseconds: 120_000_000_000)
            guard let self, self.blockingActionInFlight else { return }
            self.blockingActionInFlight = false
            self.lastClientError = "Backend action timed out. Use Refresh Status to check board state."
        }
        actionTimeoutTask = timeoutTask
        defer {
            blockingActionInFlight = false
            activeDirectRunBaseline = nil
            actionTimeoutTask?.cancel()
            actionTimeoutTask = nil
        }
        lastClientError = nil
        latestVelocitySweep = nil

        var points: [VelocitySweepPoint] = []
        var stoppedReason: String?
        for velocity in velocities {
            var trialQualities: [DirectRunQuality] = []
            for _ in 0..<trialsPerPoint {
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
                trialQualities.append(quality)
            }
            guard trialQualities.count == trialsPerPoint else {
                break
            }
            let point = aggregateVelocitySweepPoint(commandTurnsPerSecond: velocity, trialQualities: trialQualities)
            latestDirectRunQuality = point.quality
            points.append(point)
            if point.tier == .faulted {
                stoppedReason = point.quality.errorSummary ?? point.quality.headline
                break
            }
        }
        latestVelocitySweep = summarizeVelocitySweep(points: points, stoppedReason: stoppedReason, trialsPerPoint: trialsPerPoint)
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
        let hasTimedDuration = !directControlForm.outputAwareSpeedDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
        let requestedTurnsPerSecond = Double(directControlForm.outputAwareSpeedTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines))
        let requestedDuration = hasTimedDuration
            ? Double(directControlForm.outputAwareSpeedDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines))
            : nil
        let baseline = captureDirectRunBaseline(
            mode: "velocity assist",
            requestedTurnsPerSecond: requestedTurnsPerSecond,
            requestedDurationS: requestedDuration,
            releaseAfter: directControlForm.outputAwareSpeedReleaseAfter
        )
        activeDirectRunBaseline = baseline
        var args = [
            "--turns-per-second", directControlForm.outputAwareSpeedTurnsPerSecond,
            "--duration-s", directControlForm.outputAwareSpeedDurationSeconds,
            "--gear-ratio", moveForm.gearRatio,
            "--breakaway-floor-turns-s", String(assistFloorTurnsPerSecond),
            "--kick-max-duration-s", directControlForm.assistKickMaxDurationSeconds,
            "--breakaway-output-turns", directControlForm.assistBreakawayOutputTurns,
        ]
        if directControlForm.outputAwareSpeedReleaseAfter {
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

    func commandOutputAwareVelocity() async {
        guard let assistFloorTurnsPerSecond else {
            lastClientError = "Run sweep first or set a manual floor override."
            return
        }
        let startedTemporaryStream = !streamingStarted
        if startedTemporaryStream {
            await ensureStreaming()
        }
        let hasTimedDuration = !directControlForm.outputAwareSpeedDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
        let requestedTurnsPerSecond = Double(directControlForm.outputAwareSpeedTurnsPerSecond.trimmingCharacters(in: .whitespacesAndNewlines))
        let requestedDuration = hasTimedDuration
            ? Double(directControlForm.outputAwareSpeedDurationSeconds.trimmingCharacters(in: .whitespacesAndNewlines))
            : nil
        let baseline = captureDirectRunBaseline(
            mode: "output-aware speed",
            requestedTurnsPerSecond: requestedTurnsPerSecond,
            requestedDurationS: requestedDuration,
            releaseAfter: directControlForm.outputAwareSpeedReleaseAfter
        )
        activeDirectRunBaseline = baseline
        var args = [
            "--turns-per-second", directControlForm.outputAwareSpeedTurnsPerSecond,
            "--duration-s", directControlForm.outputAwareSpeedDurationSeconds,
            "--gear-ratio", moveForm.gearRatio,
            "--breakaway-floor-turns-s", String(assistFloorTurnsPerSecond),
            "--kick-max-duration-s", directControlForm.assistKickMaxDurationSeconds,
            "--breakaway-output-turns", directControlForm.assistBreakawayOutputTurns,
        ]
        if directControlForm.outputAwareSpeedReleaseAfter {
            args.append("--release-after-command")
        }
        let result = await run(action: "command-velocity-output-aware", arguments: args)
        if let result {
            latestDirectRunQuality = buildDirectRunQuality(from: result, baseline: baseline)
        }
        activeDirectRunBaseline = nil
        if startedTemporaryStream && !telemetryAutoRefresh {
            await disableStreamingIfAllowed(force: true)
        }
    }

    func commandOutputAwarePosition() async {
        guard let assistFloorTurnsPerSecond else {
            lastClientError = "Run sweep first or set a manual floor override."
            return
        }
        let startedTemporaryStream = !streamingStarted
        if startedTemporaryStream {
            await ensureStreaming()
        }
        guard let requestedOutputDegrees = Double(directControlForm.outputAwarePositionDegrees.trimmingCharacters(in: .whitespacesAndNewlines)),
              let timeoutSeconds = Double(directControlForm.outputAwarePositionTimeoutSeconds.trimmingCharacters(in: .whitespacesAndNewlines))
        else {
            lastClientError = "Output-aware position inputs must be numeric."
            return
        }
        let requestedOutputTurns = requestedOutputDegrees / 360.0
        let baseline = captureDirectRunBaseline(
            mode: "output-aware position",
            requestedMotorTurns: requestedOutputTurns * resolvedGearRatio,
            requestedOutputTurns: requestedOutputTurns,
            relative: directControlForm.outputAwarePositionRelative,
            releaseAfter: true
        )
        activeDirectRunBaseline = baseline
        let result = await run(
            action: "command-position-output-aware",
            arguments: [
                "--output-turns", String(requestedOutputTurns),
                "--timeout-s", String(timeoutSeconds),
                "--target-tolerance-turns", directControlForm.outputAwarePositionTargetToleranceTurns,
                "--target-vel-tolerance-turns-s", directControlForm.outputAwarePositionTargetVelToleranceTurnsS,
                "--gear-ratio", moveForm.gearRatio,
                "--breakaway-floor-turns-s", String(assistFloorTurnsPerSecond),
                "--kick-max-duration-s", directControlForm.assistKickMaxDurationSeconds,
                "--breakaway-output-turns", directControlForm.assistBreakawayOutputTurns,
                "--release-after-command",
            ] + (directControlForm.outputAwarePositionRelative ? ["--relative"] : [])
        )
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
            handleBackendError(error)
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
            handleBackendError(error)
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
            handleBackendError(error)
        }
    }

    func moveTwoAxesSynced() async {
        if syncAxisTargetsConflict {
            lastClientError = "Axis A and Axis B currently resolve to the same board/axis."
            return
        }
        lastClientError = nil
        blockingActionInFlight = true
        let timeoutTask = Task { @MainActor [weak self] in
            try? await Task.sleep(nanoseconds: 120_000_000_000)
            guard let self, self.blockingActionInFlight else { return }
            self.blockingActionInFlight = false
            self.lastClientError = "Backend action timed out. Use Refresh Status to check board state."
        }
        actionTimeoutTask = timeoutTask
        defer {
            blockingActionInFlight = false
            actionTimeoutTask?.cancel()
            actionTimeoutTask = nil
        }
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
            handleBackendError(error)
        }
    }

    func loadProfileEditor(name: String? = nil) async {
        let profileName = (name ?? moveForm.profileName).trimmingCharacters(in: .whitespacesAndNewlines)
        guard !profileName.isEmpty else { return }
        await run(
            action: "profile-config",
            arguments: ["--profile-name", profileName],
            countsAsBlocking: false,
            storePrimaryResponse: false,
            storeTelemetryResponse: false
        )
    }

    func saveProfileEditor() async {
        do {
            if profileEditor.isBuiltInReadOnly {
                var forked = profileEditor
                forked.forkForEditing()
                profileEditor = forked
            }
            let payload = try profileEditor.jsonPayload()
            await run(
                action: "save-profile",
                arguments: ["--profile-json", payload],
                countsAsBlocking: true,
                storePrimaryResponse: false,
                storeTelemetryResponse: false
            )
        } catch {
            handleBackendError(error)
        }
    }

    func forkLoadedProfileEditor() {
        var forked = profileEditor
        forked.forkForEditing()
        profileEditor = forked
    }

    private func resetBackendSessionState() {
        blockingActionInFlight = false
        telemetryRequestActive = false
        telemetryAutoRefresh = false
        streamingStarted = false
        backendEventTask?.cancel()
        backendEventTask = nil
        recoveryRefreshTask?.cancel()
        recoveryRefreshTask = nil
        liveMonitor.reset()
        boardState = BackendBoardState()
        response = nil
    }

    private func scheduleRecoveryRefresh(force: Bool = false) {
        guard recoveryRefreshTask == nil else { return }
        recoveryRefreshTask = Task { @MainActor [weak self] in
            guard let self else { return }
            defer { self.recoveryRefreshTask = nil }
            try? await Task.sleep(nanoseconds: 350_000_000)
            guard !self.isBusy else { return }
            if !force, self.capabilities != nil {
                return
            }
            if self.capabilities?.motion_active == true {
                return
            }
            _ = await self.run(
                action: "status",
                countsAsBlocking: false,
                storePrimaryResponse: true,
                storeTelemetryResponse: true
            )
        }
    }

    func ensureCapabilitiesLoaded() async {
        guard !isBusy, capabilities == nil else { return }
        scheduleRecoveryRefresh()
    }

    private func userFacingBackendFailureMessage(for response: BackendResponse) -> String {
        let message = (response.error?.message ?? response.message ?? "Backend request failed.")
            .trimmingCharacters(in: .whitespacesAndNewlines)

        if message.contains("MKS baseline state is invalid") {
            return "MKS startup baseline failed. The axis never became move-ready during bounded calibration. Use Make Ready or Diagnose, then inspect Backend Result for attempt details."
        }
        if message.contains("MKS baseline post-calibration state became invalid") {
            return "MKS startup became invalid again after calibration/config sync. Use Make Ready or Diagnose, then inspect Backend Result for the final health snapshot."
        }
        if message.contains("ODrive connection failed within") {
            return "Board connection failed. Check power/USB and make sure no other process is holding the drive. If the selected board serial is stale, rediscover boards."
        }
        if message.contains("A background continuous move is already active") {
            return "A background continuous move is still marked active. Wait for it to finish or refresh status if the state is stale."
        }
        if let move = response.result?.objectValue?["move"]?.objectValue {
            let failureStage = move["failure_stage"]?.stringValue?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
            let detail = move["error"]?.stringValue?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
            if !failureStage.isEmpty || !detail.isEmpty {
                let joined = [failureStage, detail].filter { !$0.isEmpty }.joined(separator: ": ")
                if !joined.isEmpty {
                    return joined
                }
            }
        }
        if message.count > 320, let prefix = message.split(separator: ";").first {
            return String(prefix)
        }
        return message
    }

    private func handleBackendError(_ error: Error) {
        if case let BackendClient.BackendClientError.backendReportedFailure(failureResponse) = error {
            response = failureResponse
            mergeProfilesIfNeeded(from: failureResponse)
            mergeProfileEditorIfNeeded(from: failureResponse)
            mergeBoardStateIfNeeded(from: failureResponse)
            lastClientError = userFacingBackendFailureMessage(for: failureResponse)
            if failureResponse.capabilities == nil || failureResponse.snapshot == nil {
                scheduleRecoveryRefresh(force: true)
            }
            if let message = failureResponse.error?.message ?? failureResponse.message,
               message.contains("Backend server exited unexpectedly")
                || message.contains("backend session restarted")
                || message.contains("Backend server failed startup handshake")
                || message.contains("Backend produced no JSON output") {
                resetBackendSessionState()
                scheduleRecoveryRefresh()
            }
            return
        }
        let message = error.localizedDescription
        lastClientError = message
        if message.contains("Backend server exited unexpectedly")
            || message.contains("backend session restarted")
            || message.contains("Backend server failed startup handshake")
            || message.contains("Backend produced no JSON output") {
            resetBackendSessionState()
            scheduleRecoveryRefresh()
        }
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
            let timeoutTask = Task { @MainActor [weak self] in
                try? await Task.sleep(nanoseconds: 45_000_000_000)
                guard let self, self.blockingActionInFlight else { return }
                self.blockingActionInFlight = false
                self.lastClientError = "Backend action timed out. Use Refresh Status to check board state."
            }
            actionTimeoutTask = timeoutTask
        }
        lastClientError = nil
        defer {
            if countsAsBlocking {
                blockingActionInFlight = false
                actionTimeoutTask?.cancel()
                actionTimeoutTask = nil
            }
        }
        do {
            let result = try await backend.run(action: action, arguments: arguments, context: requestContext())
            if storePrimaryResponse {
                response = result
            }
            if storeTelemetryResponse {
                mergeBoardStateIfNeeded(from: result)
            }
            mergeProfilesIfNeeded(from: result)
            mergeProfileEditorIfNeeded(from: result)
            liveMonitor.appendTelemetrySample(from: result, fallbackTc: snapshot?.tc)
            return result
        } catch {
            handleBackendError(error)
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
                self.streamingStarted = false
                self.backendEventTask = nil
                self.telemetryAutoRefresh = false
            }
        } catch {
            handleBackendError(error)
            streamingStarted = false
        }
    }

    private func disableStreamingIfAllowed(force: Bool = false) async {
        let motionActive = capabilities?.motion_active == true
        if !force && (telemetryAutoRefresh || motionActive) {
            return
        }
        guard streamingStarted else { return }
        do {
            try await backend.disableEventStream(context: requestContext())
        } catch {
            handleBackendError(error)
        }
        backendEventTask?.cancel()
        backendEventTask = nil
        streamingStarted = false
    }

    private func handleStreamEvent(_ event: BackendResponse) {
        switch event.action {
        case "stream-graph":
            mergeBoardStateIfNeeded(from: event)
            if telemetryAutoRefresh, let sample = event.graph_sample {
                liveMonitor.appendGraphTelemetrySample(sample)
            }
        case "stream-motion-status":
            mergeProfilesIfNeeded(from: event)
            mergeProfileEditorIfNeeded(from: event)
            mergeBoardStateIfNeeded(from: event)
            if event.ok == false || event.error != nil {
                lastClientError = userFacingBackendFailureMessage(for: event)
            } else if lastClientError?.contains("background continuous move") == true
                        || lastClientError?.contains("Background move") == true
                        || lastClientError?.contains("MKS startup baseline failed") == true {
                lastClientError = nil
            }
            if telemetryAutoRefresh {
                liveMonitor.appendTelemetrySample(from: event, fallbackTc: snapshot?.tc)
            }
            if event.capabilities?.motion_active == false {
                Task { await disableStreamingIfAllowed() }
            }
        default:
            mergeProfilesIfNeeded(from: event)
            mergeBoardStateIfNeeded(from: event)
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
        if moveForm.templateProfileSelection.isEmpty, let first = filteredTemplateProfiles.first {
            moveForm.templateProfileSelection = first.name
        }
        if !moveForm.templateProfileSelection.isEmpty,
           !filteredTemplateProfiles.contains(where: { $0.name == moveForm.templateProfileSelection }) {
            moveForm.templateProfileSelection = filteredTemplateProfiles.first?.name ?? ""
        }
        if !moveForm.manualProfileSelection.isEmpty,
           !filteredManualProfiles.contains(where: { $0.name == moveForm.manualProfileSelection }) {
            moveForm.manualProfileSelection = filteredManualProfiles.first?.name ?? ""
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
        syncProfileBrowserStateFromActiveProfile()
    }

    private func mergeBoardStateIfNeeded(from result: BackendResponse) {
        boardState.merge(from: result)
        if let motorDirection = boardState.snapshot?.motor_direction ?? result.snapshot?.motor_direction {
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
            handleBackendError(error)
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
