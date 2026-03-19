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

struct BackendResponse: Decodable {
    let ok: Bool
    let action: String
    let timestamp_s: Double?
    let device: BackendDevice?
    let message: String?
    let snapshot: BackendSnapshot?
    let diagnosis: BackendDiagnosis?
    let fact_sheet: FactSheet?
    let capabilities: BackendCapabilities?
    let available_profiles: [String]?
    let available_profile_details: [BackendProfileDetail]?
    let result: JSONValue?
    let error: BackendErrorPayload?

    var rawJSON: String = ""

    enum CodingKeys: String, CodingKey {
        case ok
        case action
        case timestamp_s
        case device
        case message
        case snapshot
        case diagnosis
        case fact_sheet
        case capabilities
        case available_profiles
        case available_profile_details
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
}

struct SliderFollowState {
    var angleDeg: Double = 0
    var liveEnabled: Bool = false
}

struct TelemetrySample: Identifiable {
    let id = UUID()
    let timestamp: Date
    let posEst: Double
    let velEst: Double
    let iqMeas: Double
    let inputPos: Double
    let estimatedMotorTorqueNm: Double?

    var trackingError: Double { inputPos - posEst }
}
