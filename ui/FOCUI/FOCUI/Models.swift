import Foundation

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
