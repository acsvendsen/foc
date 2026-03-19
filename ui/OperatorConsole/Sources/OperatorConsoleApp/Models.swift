import Foundation

private extension KeyedDecodingContainer {
    func decodeBoolishIfPresent(forKey key: Key) throws -> Bool? {
        if let value = try? decodeIfPresent(Bool.self, forKey: key) {
            return value
        }
        if let value = try? decodeIfPresent(Int.self, forKey: key) {
            return value != 0
        }
        if let value = try? decodeIfPresent(Double.self, forKey: key) {
            return value != 0
        }
        if let value = try? decodeIfPresent(String.self, forKey: key) {
            switch value.trimmingCharacters(in: .whitespacesAndNewlines).lowercased() {
            case "true", "1", "yes", "on":
                return true
            case "false", "0", "no", "off":
                return false
            default:
                return nil
            }
        }
        return nil
    }
}

struct BackendDevice: Decodable {
    let axis_index: Int?
    let serial_number: String?
    let vbus_voltage: Double?
}

struct BackendSnapshot: Decodable {
    let state: Int?
    let axis_err: Int?
    let motor_err: Int?
    let enc_err: Int?
    let ctrl_err: Int?
    let disarm_reason: Int?
    let active_errors: Int?
    let procedure_result: Int?
    let enc_ready: Bool?
    let enc_use_index: Bool?
    let enc_index_found: Bool?
    let pos_est: Double?
    let input_pos: Double?
    let pos_setpoint: Double?
    let vel_est: Double?
    let Iq_set: Double?
    let Iq_meas: Double?
    let ctrl_mode: Int?
    let input_mode: Int?
    let vel_limit: Double?
    let trap_vel: Double?
    let trap_acc: Double?
    let trap_dec: Double?
    let pos_gain: Double?
    let vel_gain: Double?
    let vel_i_gain: Double?
    let tc: Double?
    let current_lim: Double?
    let shadow_count: Int?

    enum CodingKeys: String, CodingKey {
        case state
        case axis_err
        case motor_err
        case enc_err
        case ctrl_err
        case disarm_reason
        case active_errors
        case procedure_result
        case enc_ready
        case enc_use_index
        case enc_index_found
        case pos_est
        case input_pos
        case pos_setpoint
        case vel_est
        case Iq_set
        case Iq_meas
        case ctrl_mode
        case input_mode
        case vel_limit
        case trap_vel
        case trap_acc
        case trap_dec
        case pos_gain
        case vel_gain
        case vel_i_gain
        case tc
        case current_lim
        case shadow_count
    }

    init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        state = try container.decodeIfPresent(Int.self, forKey: .state)
        axis_err = try container.decodeIfPresent(Int.self, forKey: .axis_err)
        motor_err = try container.decodeIfPresent(Int.self, forKey: .motor_err)
        enc_err = try container.decodeIfPresent(Int.self, forKey: .enc_err)
        ctrl_err = try container.decodeIfPresent(Int.self, forKey: .ctrl_err)
        disarm_reason = try container.decodeIfPresent(Int.self, forKey: .disarm_reason)
        active_errors = try container.decodeIfPresent(Int.self, forKey: .active_errors)
        procedure_result = try container.decodeIfPresent(Int.self, forKey: .procedure_result)
        enc_ready = try container.decodeBoolishIfPresent(forKey: .enc_ready)
        enc_use_index = try container.decodeBoolishIfPresent(forKey: .enc_use_index)
        enc_index_found = try container.decodeBoolishIfPresent(forKey: .enc_index_found)
        pos_est = try container.decodeIfPresent(Double.self, forKey: .pos_est)
        input_pos = try container.decodeIfPresent(Double.self, forKey: .input_pos)
        pos_setpoint = try container.decodeIfPresent(Double.self, forKey: .pos_setpoint)
        vel_est = try container.decodeIfPresent(Double.self, forKey: .vel_est)
        Iq_set = try container.decodeIfPresent(Double.self, forKey: .Iq_set)
        Iq_meas = try container.decodeIfPresent(Double.self, forKey: .Iq_meas)
        ctrl_mode = try container.decodeIfPresent(Int.self, forKey: .ctrl_mode)
        input_mode = try container.decodeIfPresent(Int.self, forKey: .input_mode)
        vel_limit = try container.decodeIfPresent(Double.self, forKey: .vel_limit)
        trap_vel = try container.decodeIfPresent(Double.self, forKey: .trap_vel)
        trap_acc = try container.decodeIfPresent(Double.self, forKey: .trap_acc)
        trap_dec = try container.decodeIfPresent(Double.self, forKey: .trap_dec)
        pos_gain = try container.decodeIfPresent(Double.self, forKey: .pos_gain)
        vel_gain = try container.decodeIfPresent(Double.self, forKey: .vel_gain)
        vel_i_gain = try container.decodeIfPresent(Double.self, forKey: .vel_i_gain)
        tc = try container.decodeIfPresent(Double.self, forKey: .tc)
        current_lim = try container.decodeIfPresent(Double.self, forKey: .current_lim)
        shadow_count = try container.decodeIfPresent(Int.self, forKey: .shadow_count)
    }
}

struct BackendDiagnosisReport: Decodable {
    let snapshot: BackendSnapshot?
    let axis_err_names: [String]?
    let motor_err_names: [String]?
    let enc_err_names: [String]?
    let ctrl_err_names: [String]?
}

struct BackendDiagnosis: Decodable {
    let severity: String?
    let diagnosis: String?
    let verdict: String?
    let verdicts: [String]?
    let commands: [String]?
    let notes: [String]?
    let report: BackendDiagnosisReport?
}

struct BackendCapabilities: Decodable {
    let can_startup: Bool?
    let can_idle: Bool?
    let can_clear_errors: Bool?
    let can_diagnose: Bool?
    let can_fact_sheet: Bool?
    let can_move_continuous: Bool?
    let can_move_continuous_aggressive: Bool?
    let can_capture_zero_here: Bool?
    let startup_ready: Bool?
    let armed: Bool?
    let idle: Bool?
    let has_latched_errors: Bool?
    let motion_active: Bool?

    enum CodingKeys: String, CodingKey {
        case can_startup
        case can_idle
        case can_clear_errors
        case can_diagnose
        case can_fact_sheet
        case can_move_continuous
        case can_move_continuous_aggressive
        case can_capture_zero_here
        case startup_ready
        case armed
        case idle
        case has_latched_errors
        case motion_active
    }

    init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        can_startup = try container.decodeBoolishIfPresent(forKey: .can_startup)
        can_idle = try container.decodeBoolishIfPresent(forKey: .can_idle)
        can_clear_errors = try container.decodeBoolishIfPresent(forKey: .can_clear_errors)
        can_diagnose = try container.decodeBoolishIfPresent(forKey: .can_diagnose)
        can_fact_sheet = try container.decodeBoolishIfPresent(forKey: .can_fact_sheet)
        can_move_continuous = try container.decodeBoolishIfPresent(forKey: .can_move_continuous)
        can_move_continuous_aggressive = try container.decodeBoolishIfPresent(forKey: .can_move_continuous_aggressive)
        can_capture_zero_here = try container.decodeBoolishIfPresent(forKey: .can_capture_zero_here)
        startup_ready = try container.decodeBoolishIfPresent(forKey: .startup_ready)
        armed = try container.decodeBoolishIfPresent(forKey: .armed)
        idle = try container.decodeBoolishIfPresent(forKey: .idle)
        has_latched_errors = try container.decodeBoolishIfPresent(forKey: .has_latched_errors)
        motion_active = try container.decodeBoolishIfPresent(forKey: .motion_active)
    }
}

struct FactRow: Decodable, Identifiable {
    let label: String
    let kind: String
    let value: JSONValue?
    let note: String

    var id: String { "\(kind)-\(label)" }
}

struct FactSheet: Decodable {
    let measured_live: [FactRow]?
    let configured: [FactRow]?
    let inferred: [FactRow]?
    let unknown: [FactRow]?
    let quick_checks: [FactRow]?
}

enum JSONValue: Decodable, CustomStringConvertible {
    case string(String)
    case number(Double)
    case bool(Bool)
    case object([String: JSONValue])
    case array([JSONValue])
    case null

    init(from decoder: Decoder) throws {
        let container = try decoder.singleValueContainer()
        if container.decodeNil() {
            self = .null
        } else if let value = try? container.decode(Bool.self) {
            self = .bool(value)
        } else if let value = try? container.decode(Int.self) {
            self = .number(Double(value))
        } else if let value = try? container.decode(Double.self) {
            self = .number(value)
        } else if let value = try? container.decode(String.self) {
            self = .string(value)
        } else if let value = try? container.decode([String: JSONValue].self) {
            self = .object(value)
        } else if let value = try? container.decode([JSONValue].self) {
            self = .array(value)
        } else {
            throw DecodingError.typeMismatch(JSONValue.self, DecodingError.Context(codingPath: decoder.codingPath, debugDescription: "Unsupported JSON value"))
        }
    }

    var description: String {
        switch self {
        case .string(let value): return value
        case .number(let value):
            if value.rounded() == value {
                return String(format: "%.0f", value)
            }
            return String(format: "%.6g", value)
        case .bool(let value): return value ? "true" : "false"
        case .object(let value):
            let pairs = value.keys.sorted().compactMap { key in
                value[key].map { "\(key): \($0.description)" }
            }
            return "{\(pairs.joined(separator: ", "))}"
        case .array(let value):
            return "[\(value.map(\.description).joined(separator: ", "))]"
        case .null:
            return "unknown"
        }
    }
}

struct BackendErrorPayload: Decodable {
    let type: String?
    let message: String?
    let traceback: String?
}

struct BackendProfileDetail: Decodable, Identifiable {
    let name: String
    let notes: String?
    let limitations: [String]?
    let source: String?

    var id: String { name }
}

struct BackendProfileEditor: Decodable {
    let name: String
    let notes: String?
    let source: String?
    let load_mode: String?
    let require_repeatability: Bool?
    let stop_on_frame_jump: Bool?
    let stop_on_hard_fault: Bool?
    let limitations: [String]?
    let current_lim: Double?
    let enable_overspeed_error: Bool?
    let pos_gain: Double?
    let vel_gain: Double?
    let vel_i_gain: Double?
    let trap_vel: Double?
    let trap_acc: Double?
    let trap_dec: Double?
    let vel_limit: Double?
    let vel_limit_tolerance: Double?
    let stiction_kick_nm: Double?
    let target_tolerance_turns: Double?
    let target_vel_tolerance_turns_s: Double?
    let timeout_s: Double?
    let min_delta_turns: Double?
    let settle_s: Double?
    let quiet_hold_enable: Bool?
    let quiet_hold_s: Double?
    let quiet_hold_pos_gain_scale: Double?
    let quiet_hold_vel_gain_scale: Double?
    let quiet_hold_vel_i_gain: Double?
    let quiet_hold_vel_limit_scale: Double?
    let quiet_hold_persist: Bool?
    let quiet_hold_reanchor_err_turns: Double?
    let fail_to_idle: Bool?

    enum CodingKeys: String, CodingKey {
        case name
        case notes
        case source
        case load_mode
        case require_repeatability
        case stop_on_frame_jump
        case stop_on_hard_fault
        case limitations
        case current_lim
        case enable_overspeed_error
        case pos_gain
        case vel_gain
        case vel_i_gain
        case trap_vel
        case trap_acc
        case trap_dec
        case vel_limit
        case vel_limit_tolerance
        case stiction_kick_nm
        case target_tolerance_turns
        case target_vel_tolerance_turns_s
        case timeout_s
        case min_delta_turns
        case settle_s
        case quiet_hold_enable
        case quiet_hold_s
        case quiet_hold_pos_gain_scale
        case quiet_hold_vel_gain_scale
        case quiet_hold_vel_i_gain
        case quiet_hold_vel_limit_scale
        case quiet_hold_persist
        case quiet_hold_reanchor_err_turns
        case fail_to_idle
    }

    init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        name = try container.decode(String.self, forKey: .name)
        notes = try container.decodeIfPresent(String.self, forKey: .notes)
        source = try container.decodeIfPresent(String.self, forKey: .source)
        load_mode = try container.decodeIfPresent(String.self, forKey: .load_mode)
        require_repeatability = try container.decodeBoolishIfPresent(forKey: .require_repeatability)
        stop_on_frame_jump = try container.decodeBoolishIfPresent(forKey: .stop_on_frame_jump)
        stop_on_hard_fault = try container.decodeBoolishIfPresent(forKey: .stop_on_hard_fault)
        limitations = try container.decodeIfPresent([String].self, forKey: .limitations)
        current_lim = try container.decodeIfPresent(Double.self, forKey: .current_lim)
        enable_overspeed_error = try container.decodeBoolishIfPresent(forKey: .enable_overspeed_error)
        pos_gain = try container.decodeIfPresent(Double.self, forKey: .pos_gain)
        vel_gain = try container.decodeIfPresent(Double.self, forKey: .vel_gain)
        vel_i_gain = try container.decodeIfPresent(Double.self, forKey: .vel_i_gain)
        trap_vel = try container.decodeIfPresent(Double.self, forKey: .trap_vel)
        trap_acc = try container.decodeIfPresent(Double.self, forKey: .trap_acc)
        trap_dec = try container.decodeIfPresent(Double.self, forKey: .trap_dec)
        vel_limit = try container.decodeIfPresent(Double.self, forKey: .vel_limit)
        vel_limit_tolerance = try container.decodeIfPresent(Double.self, forKey: .vel_limit_tolerance)
        stiction_kick_nm = try container.decodeIfPresent(Double.self, forKey: .stiction_kick_nm)
        target_tolerance_turns = try container.decodeIfPresent(Double.self, forKey: .target_tolerance_turns)
        target_vel_tolerance_turns_s = try container.decodeIfPresent(Double.self, forKey: .target_vel_tolerance_turns_s)
        timeout_s = try container.decodeIfPresent(Double.self, forKey: .timeout_s)
        min_delta_turns = try container.decodeIfPresent(Double.self, forKey: .min_delta_turns)
        settle_s = try container.decodeIfPresent(Double.self, forKey: .settle_s)
        quiet_hold_enable = try container.decodeBoolishIfPresent(forKey: .quiet_hold_enable)
        quiet_hold_s = try container.decodeIfPresent(Double.self, forKey: .quiet_hold_s)
        quiet_hold_pos_gain_scale = try container.decodeIfPresent(Double.self, forKey: .quiet_hold_pos_gain_scale)
        quiet_hold_vel_gain_scale = try container.decodeIfPresent(Double.self, forKey: .quiet_hold_vel_gain_scale)
        quiet_hold_vel_i_gain = try container.decodeIfPresent(Double.self, forKey: .quiet_hold_vel_i_gain)
        quiet_hold_vel_limit_scale = try container.decodeIfPresent(Double.self, forKey: .quiet_hold_vel_limit_scale)
        quiet_hold_persist = try container.decodeBoolishIfPresent(forKey: .quiet_hold_persist)
        quiet_hold_reanchor_err_turns = try container.decodeIfPresent(Double.self, forKey: .quiet_hold_reanchor_err_turns)
        fail_to_idle = try container.decodeBoolishIfPresent(forKey: .fail_to_idle)
    }
}

struct GraphTelemetrySamplePayload: Decodable {
    let timestamp_s: Double?
    let pos_est: Double?
    let vel_est: Double?
    let Iq_meas: Double?
    let input_pos: Double?
    let tracking_err_turns: Double?
    let estimated_motor_torque_nm: Double?
}

struct BackendResponse: Decodable {
    let ok: Bool
    let action: String
    let request_id: String?
    let timestamp_s: Double?
    let device: BackendDevice?
    let message: String?
    let snapshot: BackendSnapshot?
    let diagnosis: BackendDiagnosis?
    let fact_sheet: FactSheet?
    let capabilities: BackendCapabilities?
    let available_profiles: [String]?
    let available_profile_details: [BackendProfileDetail]?
    let profile_editor: BackendProfileEditor?
    let graph_sample: GraphTelemetrySamplePayload?
    let result: JSONValue?
    let error: BackendErrorPayload?

    var rawJSON: String = ""

    enum CodingKeys: String, CodingKey {
        case ok
        case action
        case request_id
        case timestamp_s
        case device
        case message
        case snapshot
        case diagnosis
        case fact_sheet
        case capabilities
        case available_profiles
        case available_profile_details
        case profile_editor
        case graph_sample
        case result
        case error
    }

    var resultSummary: String {
        if let result {
            return result.description
        }
        if let message {
            return message
        }
        return ok ? "OK" : "No result"
    }
}

struct MoveFormState {
    var angleDeg: String = "10"
    var angleSpace: String = "gearbox_output"
    var relativeToCurrent: Bool = true
    var zeroTurnsMotor: String = ""
    var gearRatio: String = "25"
    var timeoutSeconds: String = ""
    var profileName: String = "gearbox_output_continuous_quiet_20260309"
    var releaseAfterMove: Bool = false
}

struct ProfileEditorFormState {
    var loadedProfileName: String = ""
    var name: String = ""
    var notes: String = ""
    var source: String = "focui_manual_editor"
    var loadMode: String = "loaded"
    var requireRepeatability: Bool = false
    var stopOnFrameJump: Bool = true
    var stopOnHardFault: Bool = true
    var limitationsText: String = ""

    var currentLim: String = "6.5"
    var enableOverspeedError: Bool = false
    var posGain: String = "12"
    var velGain: String = "0.22"
    var velIGain: String = "0"
    var trapVel: String = "0.28"
    var trapAcc: String = "0.32"
    var trapDec: String = "0.32"
    var velLimit: String = "0.40"
    var velLimitTolerance: String = "4.0"
    var stictionKickNm: String = "0"
    var targetToleranceTurns: String = "0.03"
    var targetVelToleranceTurnsS: String = "0.20"
    var timeoutS: String = "8.0"
    var minDeltaTurns: String = "0.0015"
    var settleS: String = "0.08"

    var quietHoldEnable: Bool = true
    var quietHoldS: String = "0.06"
    var quietHoldPosGainScale: String = "0.45"
    var quietHoldVelGainScale: String = "0.70"
    var quietHoldVelIGain: String = "0"
    var quietHoldVelLimitScale: String = "0.50"
    var quietHoldPersist: Bool = true
    var quietHoldReanchorErrTurns: String = "0.035"
    var quietHoldReanchorDisabled: Bool = false
    var failToIdle: Bool = false

    init() {}

    init(editor: BackendProfileEditor) {
        loadedProfileName = editor.name
        name = editor.name
        notes = editor.notes ?? ""
        source = editor.source ?? "focui_manual_editor"
        loadMode = editor.load_mode ?? "loaded"
        requireRepeatability = editor.require_repeatability ?? false
        stopOnFrameJump = editor.stop_on_frame_jump ?? true
        stopOnHardFault = editor.stop_on_hard_fault ?? true
        limitationsText = (editor.limitations ?? []).joined(separator: "\n")

        currentLim = Self.format(editor.current_lim, fallback: "6.5")
        enableOverspeedError = editor.enable_overspeed_error ?? false
        posGain = Self.format(editor.pos_gain, fallback: "12")
        velGain = Self.format(editor.vel_gain, fallback: "0.22")
        velIGain = Self.format(editor.vel_i_gain, fallback: "0")
        trapVel = Self.format(editor.trap_vel, fallback: "0.28")
        trapAcc = Self.format(editor.trap_acc, fallback: "0.32")
        trapDec = Self.format(editor.trap_dec, fallback: "0.32")
        velLimit = Self.format(editor.vel_limit, fallback: "0.40")
        velLimitTolerance = Self.format(editor.vel_limit_tolerance, fallback: "4.0")
        stictionKickNm = Self.format(editor.stiction_kick_nm, fallback: "0")
        targetToleranceTurns = Self.format(editor.target_tolerance_turns, fallback: "0.03")
        targetVelToleranceTurnsS = Self.format(editor.target_vel_tolerance_turns_s, fallback: "0.20")
        timeoutS = Self.format(editor.timeout_s, fallback: "8.0")
        minDeltaTurns = Self.format(editor.min_delta_turns, fallback: "0.0015")
        settleS = Self.format(editor.settle_s, fallback: "0.08")

        quietHoldEnable = editor.quiet_hold_enable ?? true
        quietHoldS = Self.format(editor.quiet_hold_s, fallback: "0.06")
        quietHoldPosGainScale = Self.format(editor.quiet_hold_pos_gain_scale, fallback: "0.45")
        quietHoldVelGainScale = Self.format(editor.quiet_hold_vel_gain_scale, fallback: "0.70")
        quietHoldVelIGain = Self.format(editor.quiet_hold_vel_i_gain, fallback: "0")
        quietHoldVelLimitScale = Self.format(editor.quiet_hold_vel_limit_scale, fallback: "0.50")
        quietHoldPersist = editor.quiet_hold_persist ?? true
        quietHoldReanchorDisabled = (editor.quiet_hold_reanchor_err_turns == nil)
        quietHoldReanchorErrTurns = Self.format(editor.quiet_hold_reanchor_err_turns, fallback: "0.035")
        failToIdle = editor.fail_to_idle ?? false
    }

    private static func format(_ value: Double?, fallback: String) -> String {
        guard let value else { return fallback }
        return String(format: "%.6g", value)
    }

    func normalizedLimitations() -> [String] {
        limitationsText
            .split(whereSeparator: \.isNewline)
            .map { String($0).trimmingCharacters(in: .whitespacesAndNewlines) }
            .filter { !$0.isEmpty }
    }

    func jsonPayload() throws -> String {
        let payload = ProfileEditorPayload(
            name: name.trimmingCharacters(in: .whitespacesAndNewlines),
            notes: notes,
            source: source.trimmingCharacters(in: .whitespacesAndNewlines),
            load_mode: loadMode.trimmingCharacters(in: .whitespacesAndNewlines),
            require_repeatability: requireRepeatability,
            stop_on_frame_jump: stopOnFrameJump,
            stop_on_hard_fault: stopOnHardFault,
            limitations: normalizedLimitations(),
            current_lim: Double(currentLim) ?? 6.5,
            enable_overspeed_error: enableOverspeedError,
            pos_gain: Double(posGain) ?? 12.0,
            vel_gain: Double(velGain) ?? 0.22,
            vel_i_gain: Double(velIGain) ?? 0.0,
            trap_vel: Double(trapVel) ?? 0.28,
            trap_acc: Double(trapAcc) ?? 0.32,
            trap_dec: Double(trapDec) ?? 0.32,
            vel_limit: Double(velLimit) ?? 0.40,
            vel_limit_tolerance: Double(velLimitTolerance) ?? 4.0,
            stiction_kick_nm: Double(stictionKickNm) ?? 0.0,
            target_tolerance_turns: Double(targetToleranceTurns) ?? 0.03,
            target_vel_tolerance_turns_s: Double(targetVelToleranceTurnsS) ?? 0.20,
            timeout_s: Double(timeoutS) ?? 8.0,
            min_delta_turns: Double(minDeltaTurns) ?? 0.0015,
            settle_s: Double(settleS) ?? 0.08,
            quiet_hold_enable: quietHoldEnable,
            quiet_hold_s: Double(quietHoldS) ?? 0.06,
            quiet_hold_pos_gain_scale: Double(quietHoldPosGainScale) ?? 0.45,
            quiet_hold_vel_gain_scale: Double(quietHoldVelGainScale) ?? 0.70,
            quiet_hold_vel_i_gain: Double(quietHoldVelIGain) ?? 0.0,
            quiet_hold_vel_limit_scale: Double(quietHoldVelLimitScale) ?? 0.50,
            quiet_hold_persist: quietHoldPersist,
            quiet_hold_reanchor_err_turns: quietHoldReanchorDisabled ? nil : (Double(quietHoldReanchorErrTurns) ?? 0.035),
            fail_to_idle: failToIdle
        )
        let data = try JSONEncoder().encode(payload)
        guard let text = String(data: data, encoding: .utf8) else {
            throw NSError(domain: "ProfileEditorFormState", code: 1, userInfo: [NSLocalizedDescriptionKey: "Failed to encode profile payload as UTF-8"])
        }
        return text
    }
}

private struct ProfileEditorPayload: Encodable {
    let name: String
    let notes: String
    let source: String
    let load_mode: String
    let require_repeatability: Bool
    let stop_on_frame_jump: Bool
    let stop_on_hard_fault: Bool
    let limitations: [String]
    let current_lim: Double
    let enable_overspeed_error: Bool
    let pos_gain: Double
    let vel_gain: Double
    let vel_i_gain: Double
    let trap_vel: Double
    let trap_acc: Double
    let trap_dec: Double
    let vel_limit: Double
    let vel_limit_tolerance: Double
    let stiction_kick_nm: Double
    let target_tolerance_turns: Double
    let target_vel_tolerance_turns_s: Double
    let timeout_s: Double
    let min_delta_turns: Double
    let settle_s: Double
    let quiet_hold_enable: Bool
    let quiet_hold_s: Double
    let quiet_hold_pos_gain_scale: Double
    let quiet_hold_vel_gain_scale: Double
    let quiet_hold_vel_i_gain: Double
    let quiet_hold_vel_limit_scale: Double
    let quiet_hold_persist: Bool
    let quiet_hold_reanchor_err_turns: Double?
    let fail_to_idle: Bool
}

struct SliderFollowState {
    var angleDeg: Double = 0
    var liveEnabled: Bool = false
}

struct TelemetrySample: Identifiable {
    let id: Double
    let timestampS: Double
    let posEst: Double
    let velEst: Double
    let iqMeas: Double
    let inputPos: Double
    let estimatedMotorTorqueNm: Double?

    var trackingError: Double { inputPos - posEst }
}
