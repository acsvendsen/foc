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
        case backendExited(String)
        case emptyOutput
        case invalidUTF8
        case invalidJSON(String)
        case invalidHandshake(String)

        var errorDescription: String? {
            switch self {
            case .backendNotFound(let path):
                return "Backend script not found at \(path)"
            case .backendExited(let detail):
                return "Backend server exited unexpectedly: \(detail)"
            case .emptyOutput:
                return "Backend produced no JSON output"
            case .invalidUTF8:
                return "Backend output was not valid UTF-8"
            case .invalidJSON(let raw):
                return "Backend returned invalid JSON: \(raw)"
            case .invalidHandshake(let raw):
                return "Backend server failed startup handshake: \(raw)"
            }
        }
    }

    private static let session = BackendProcessSession()

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

    private func requestArguments(arguments: [String], context: RequestContext) -> [String] {
        var args = [
            "--axis-index", String(context.axisIndex),
            "--kv-est", context.kvEstimate,
            "--line-line-r-ohm", context.lineLineROhm,
            "--settle-s", context.settleSeconds,
        ]
        if context.debug {
            args.append("--debug")
        }
        args.append(contentsOf: arguments)
        return args
    }

    func run(action: String, arguments: [String], context: RequestContext) async throws -> BackendResponse {
        let repoRootURL = URL(fileURLWithPath: context.repoRoot)
        let backendURL = repoRootURL.appendingPathComponent("ui/backend/odrive_operator_backend.py")
        guard FileManager.default.fileExists(atPath: backendURL.path) else {
            throw BackendClientError.backendNotFound(backendURL.path)
        }
        let pythonExecutable = detectPythonExecutable(repoRootURL: repoRootURL)
        let requestArgs = requestArguments(arguments: arguments, context: context)
        return try await Self.session.send(
            action: action,
            arguments: requestArgs,
            repoRootURL: repoRootURL,
            backendURL: backendURL,
            pythonExecutable: pythonExecutable
        )
    }
}

private actor BackendProcessSession {
    private struct ServerRequest: Encodable {
        let id: String
        let action: String
        let arguments: [String]
    }

    private struct ProcessHandle {
        let repoRootPath: String
        let backendPath: String
        let pythonExecutable: String
        let process: Process
        let stdin: FileHandle
        let stdout: FileHandle
        let stderr: FileHandle
    }

    private var handle: ProcessHandle?

    func send(
        action: String,
        arguments: [String],
        repoRootURL: URL,
        backendURL: URL,
        pythonExecutable: String
    ) async throws -> BackendResponse {
        let activeHandle = try startIfNeeded(
            repoRootURL: repoRootURL,
            backendURL: backendURL,
            pythonExecutable: pythonExecutable
        )

        let request = ServerRequest(
            id: UUID().uuidString,
            action: action,
            arguments: arguments
        )
        let requestData = try JSONEncoder().encode(request)
        var line = requestData
        line.append(0x0A)
        activeHandle.stdin.write(line)

        let rawLine = try readLine(stdout: activeHandle.stdout, stderr: activeHandle.stderr, process: activeHandle.process)
        guard let responseData = rawLine.data(using: .utf8) else {
            throw BackendClient.BackendClientError.invalidUTF8
        }
        do {
            var response = try JSONDecoder().decode(BackendResponse.self, from: responseData)
            response.rawJSON = rawLine
            return response
        } catch {
            throw BackendClient.BackendClientError.invalidJSON("\(error)\n\n\(rawLine)")
        }
    }

    private func startIfNeeded(
        repoRootURL: URL,
        backendURL: URL,
        pythonExecutable: String
    ) throws -> ProcessHandle {
        if let handle,
           handle.process.isRunning,
           handle.repoRootPath == repoRootURL.path,
           handle.backendPath == backendURL.path,
           handle.pythonExecutable == pythonExecutable {
            return handle
        }

        stopExisting()

        let process = Process()
        process.currentDirectoryURL = repoRootURL
        process.executableURL = URL(fileURLWithPath: "/usr/bin/env")
        process.arguments = [pythonExecutable, backendURL.path, "serve"]

        let stdoutPipe = Pipe()
        let stderrPipe = Pipe()
        let stdinPipe = Pipe()
        process.standardOutput = stdoutPipe
        process.standardError = stderrPipe
        process.standardInput = stdinPipe

        try process.run()

        let newHandle = ProcessHandle(
            repoRootPath: repoRootURL.path,
            backendPath: backendURL.path,
            pythonExecutable: pythonExecutable,
            process: process,
            stdin: stdinPipe.fileHandleForWriting,
            stdout: stdoutPipe.fileHandleForReading,
            stderr: stderrPipe.fileHandleForReading
        )

        let rawHandshake = try readLine(stdout: newHandle.stdout, stderr: newHandle.stderr, process: newHandle.process)
        guard let handshakeData = rawHandshake.data(using: .utf8) else {
            throw BackendClient.BackendClientError.invalidUTF8
        }
        let handshake = try JSONDecoder().decode(BackendResponse.self, from: handshakeData)
        guard handshake.ok, handshake.action == "serve" else {
            throw BackendClient.BackendClientError.invalidHandshake(rawHandshake)
        }

        handle = newHandle
        return newHandle
    }

    private func stopExisting() {
        guard let handle else { return }
        if handle.process.isRunning {
            handle.process.terminate()
        }
        self.handle = nil
    }

    private func readLine(stdout: FileHandle, stderr: FileHandle, process: Process) throws -> String {
        var buffer = Data()
        while true {
            let chunk = try stdout.read(upToCount: 1) ?? Data()
            if chunk.isEmpty {
                let stderrText = try readStderr(stderr)
                if process.isRunning {
                    throw BackendClient.BackendClientError.emptyOutput
                }
                throw BackendClient.BackendClientError.backendExited(stderrText.isEmpty ? "no stderr output" : stderrText)
            }
            if chunk == Data([0x0A]) {
                break
            }
            buffer.append(chunk)
        }
        guard let text = String(data: buffer, encoding: .utf8) else {
            throw BackendClient.BackendClientError.invalidUTF8
        }
        return text
    }

    private func readStderr(_ stderr: FileHandle) throws -> String {
        let data = try stderr.readToEnd() ?? Data()
        return String(data: data, encoding: .utf8) ?? ""
    }
}
