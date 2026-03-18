import Foundation

struct BackendClient {
    struct RequestContext {
        var repoRoot: String
        var axisIndex: Int
        var kvEstimate: String
        var lineLineROhm: String
        var settleSeconds: String
        var debug: Bool
    }

    enum BackendClientError: Error, LocalizedError {
        case backendNotFound(String)
        case emptyOutput
        case invalidUTF8
        case invalidJSON(String)

        var errorDescription: String? {
            switch self {
            case .backendNotFound(let path):
                return "Backend script not found at \(path)"
            case .emptyOutput:
                return "Backend produced no JSON output"
            case .invalidUTF8:
                return "Backend output was not valid UTF-8"
            case .invalidJSON(let raw):
                return "Backend returned invalid JSON: \(raw)"
            }
        }
    }

    private func firstRepoRoot(in candidates: [URL]) -> String? {
        let fileManager = FileManager.default
        for base in candidates {
            var candidate = base
            for _ in 0..<8 {
                let backend = candidate.appendingPathComponent("ui/backend/odrive_operator_backend.py")
                if fileManager.fileExists(atPath: backend.path) {
                    return candidate.path
                }
                let parent = candidate.deletingLastPathComponent()
                if parent.path == candidate.path {
                    break
                }
                candidate = parent
            }
        }
        return nil
    }

    func detectRepoRoot() -> String {
        let fileManager = FileManager.default

        if let envRoot = ProcessInfo.processInfo.environment["ROBOT_REPO_ROOT"], !envRoot.isEmpty {
            let envURL = URL(fileURLWithPath: envRoot)
            if let found = firstRepoRoot(in: [envURL]) {
                return found
            }
        }

        let cwdURL = URL(fileURLWithPath: fileManager.currentDirectoryPath)
        let sourceURL = URL(fileURLWithPath: #filePath)
        if let found = firstRepoRoot(in: [cwdURL, sourceURL]) {
            return found
        }

        return cwdURL.path
    }

    private func detectPythonExecutable(repoRootURL: URL) -> String {
        let fileManager = FileManager.default
        let venvPython = repoRootURL.appendingPathComponent(".venv/bin/python")
        if fileManager.isExecutableFile(atPath: venvPython.path) {
            return venvPython.path
        }
        return "python3"
    }

    func run(action: String, arguments: [String], context: RequestContext) async throws -> BackendResponse {
        try await Task.detached(priority: .userInitiated) {
            let repoRootURL = URL(fileURLWithPath: context.repoRoot)
            let backendURL = repoRootURL.appendingPathComponent("ui/backend/odrive_operator_backend.py")
            guard FileManager.default.fileExists(atPath: backendURL.path) else {
                throw BackendClientError.backendNotFound(backendURL.path)
            }

            let process = Process()
            process.currentDirectoryURL = repoRootURL
            process.executableURL = URL(fileURLWithPath: "/usr/bin/env")
            let pythonExecutable = detectPythonExecutable(repoRootURL: repoRootURL)
            var args = [
                pythonExecutable,
                backendURL.path,
                action,
                "--axis-index", String(context.axisIndex),
                "--kv-est", context.kvEstimate,
                "--line-line-r-ohm", context.lineLineROhm,
                "--settle-s", context.settleSeconds,
            ]
            if context.debug {
                args.append("--debug")
            }
            args.append(contentsOf: arguments)
            process.arguments = args

            let stdoutPipe = Pipe()
            let stderrPipe = Pipe()
            process.standardOutput = stdoutPipe
            process.standardError = stderrPipe

            try process.run()
            process.waitUntilExit()

            let stdoutData = stdoutPipe.fileHandleForReading.readDataToEndOfFile()
            let stderrData = stderrPipe.fileHandleForReading.readDataToEndOfFile()

            guard !stdoutData.isEmpty else {
                if let stderrText = String(data: stderrData, encoding: .utf8), !stderrText.isEmpty {
                    throw BackendClientError.invalidJSON(stderrText)
                }
                throw BackendClientError.emptyOutput
            }
            guard let stdoutText = String(data: stdoutData, encoding: .utf8) else {
                throw BackendClientError.invalidUTF8
            }

            let decoder = JSONDecoder()
            do {
                var response = try decoder.decode(BackendResponse.self, from: stdoutData)
                response.rawJSON = stdoutText
                return response
            } catch {
                throw BackendClientError.invalidJSON(stdoutText)
            }
        }.value
    }
}
