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

    func ensureEventStream(context: RequestContext, intervalMs: Int = 40) async throws -> AsyncStream<BackendResponse> {
        let repoRootURL = URL(fileURLWithPath: context.repoRoot)
        let backendURL = repoRootURL.appendingPathComponent("ui/backend/odrive_operator_backend.py")
        guard FileManager.default.fileExists(atPath: backendURL.path) else {
            throw BackendClientError.backendNotFound(backendURL.path)
        }
        let pythonExecutable = detectPythonExecutable(repoRootURL: repoRootURL)
        let requestArgs = requestArguments(
            arguments: ["--interval-ms", String(max(20, intervalMs))],
            context: context
        )
        return try await Self.session.ensureEventStream(
            subscribeArguments: requestArgs,
            repoRootURL: repoRootURL,
            backendURL: backendURL,
            pythonExecutable: pythonExecutable
        )
    }

    func disableEventStream(context: RequestContext) async throws {
        let repoRootURL = URL(fileURLWithPath: context.repoRoot)
        let backendURL = repoRootURL.appendingPathComponent("ui/backend/odrive_operator_backend.py")
        guard FileManager.default.fileExists(atPath: backendURL.path) else {
            throw BackendClientError.backendNotFound(backendURL.path)
        }
        let pythonExecutable = detectPythonExecutable(repoRootURL: repoRootURL)
        let requestArgs = requestArguments(arguments: [], context: context)
        try await Self.session.disableEventStream(
            unsubscribeArguments: requestArgs,
            repoRootURL: repoRootURL,
            backendURL: backendURL,
            pythonExecutable: pythonExecutable
        )
    }
}

private actor BackendProcessSession {
    private static let maxRetainedRawJSONCharacters = 16_384

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
    private var readerTask: Task<Void, Never>?
    private var pendingResponses: [String: CheckedContinuation<BackendResponse, Error>] = [:]
    private var eventContinuation: AsyncStream<BackendResponse>.Continuation?
    private var streamSubscriptionKey: String?

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

        return try await withCheckedThrowingContinuation { continuation in
            pendingResponses[request.id] = continuation
            do {
                let requestData = try JSONEncoder().encode(request)
                var line = requestData
                line.append(0x0A)
                try activeHandle.stdin.write(contentsOf: line)
            } catch {
                pendingResponses.removeValue(forKey: request.id)
                continuation.resume(throwing: error)
            }
        }
    }

    func ensureEventStream(
        subscribeArguments: [String],
        repoRootURL: URL,
        backendURL: URL,
        pythonExecutable: String
    ) async throws -> AsyncStream<BackendResponse> {
        _ = try startIfNeeded(
            repoRootURL: repoRootURL,
            backendURL: backendURL,
            pythonExecutable: pythonExecutable
        )

        let subscriptionKey = [repoRootURL.path, backendURL.path, pythonExecutable, subscribeArguments.joined(separator: "\u{1F}")]
            .joined(separator: "|")

        let stream = AsyncStream<BackendResponse>(bufferingPolicy: .bufferingNewest(32)) { continuation in
            self.eventContinuation = continuation
            continuation.onTermination = { @Sendable _ in
                Task { await self.clearEventContinuation() }
            }
        }

        if streamSubscriptionKey != subscriptionKey {
            _ = try await send(
                action: "stream-subscribe",
                arguments: subscribeArguments,
                repoRootURL: repoRootURL,
                backendURL: backendURL,
                pythonExecutable: pythonExecutable
            )
            streamSubscriptionKey = subscriptionKey
        }

        return stream
    }

    func disableEventStream(
        unsubscribeArguments: [String],
        repoRootURL: URL,
        backendURL: URL,
        pythonExecutable: String
    ) async throws {
        guard streamSubscriptionKey != nil else {
            return
        }
        _ = try startIfNeeded(
            repoRootURL: repoRootURL,
            backendURL: backendURL,
            pythonExecutable: pythonExecutable
        )
        _ = try await send(
            action: "stream-unsubscribe",
            arguments: unsubscribeArguments,
            repoRootURL: repoRootURL,
            backendURL: backendURL,
            pythonExecutable: pythonExecutable
        )
        streamSubscriptionKey = nil
        eventContinuation?.finish()
        eventContinuation = nil
        readerTask?.cancel()
        readerTask = nil
        if let handle, handle.process.isRunning {
            handle.process.terminate()
        }
        handle = nil
    }

    private func clearEventContinuation() {
        eventContinuation = nil
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

        stopExisting(reason: BackendClient.BackendClientError.backendExited("backend session restarted"))

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

        let rawHandshake = try Self.readLine(stdout: newHandle.stdout, stderr: newHandle.stderr, process: newHandle.process)
        guard let handshakeData = rawHandshake.data(using: .utf8) else {
            throw BackendClient.BackendClientError.invalidUTF8
        }
        let handshake = try JSONDecoder().decode(BackendResponse.self, from: handshakeData)
        guard handshake.ok, handshake.action == "serve" else {
            throw BackendClient.BackendClientError.invalidHandshake(rawHandshake)
        }

        handle = newHandle
        startReaderTask(for: newHandle)
        return newHandle
    }

    private func stopExisting(reason: Error) {
        readerTask?.cancel()
        readerTask = nil
        if let handle, handle.process.isRunning {
            handle.process.terminate()
        }
        handle = nil
        streamSubscriptionKey = nil
        failAllPending(with: reason)
        eventContinuation?.finish()
        eventContinuation = nil
    }

    private func failAllPending(with error: Error) {
        let continuations = pendingResponses.values
        pendingResponses.removeAll()
        for continuation in continuations {
            continuation.resume(throwing: error)
        }
    }

    private func startReaderTask(for handle: ProcessHandle) {
        readerTask?.cancel()
        let processID = ObjectIdentifier(handle.process)
        readerTask = Task.detached(priority: .utility) { [stdout = handle.stdout, stderr = handle.stderr, process = handle.process, owner = self] in
            var buffer = Data()
            do {
                while !Task.isCancelled {
                    let lines: [String] = try autoreleasepool {
                        let chunk = try stdout.read(upToCount: 4096) ?? Data()
                        if chunk.isEmpty {
                            if !buffer.isEmpty {
                                guard let rawLine = String(data: buffer, encoding: .utf8) else {
                                    throw BackendClient.BackendClientError.invalidUTF8
                                }
                                buffer.removeAll(keepingCapacity: false)
                                return [rawLine]
                            }
                            let stderrText = try BackendProcessSession.readStderr(stderr)
                            if process.isRunning {
                                throw BackendClient.BackendClientError.emptyOutput
                            }
                            throw BackendClient.BackendClientError.backendExited(stderrText.isEmpty ? "no stderr output" : stderrText)
                        }
                        buffer.append(chunk)
                        var lines: [String] = []
                        while let newlineIndex = buffer.firstIndex(of: 0x0A) {
                            let lineData = buffer.prefix(upTo: newlineIndex)
                            buffer.removeSubrange(...newlineIndex)
                            guard let rawLine = String(data: lineData, encoding: .utf8) else {
                                throw BackendClient.BackendClientError.invalidUTF8
                            }
                            lines.append(rawLine)
                        }
                        return lines
                    }
                    for rawLine in lines {
                        await owner.routeIncomingLine(rawLine, processID: processID)
                    }
                }
            } catch {
                await owner.handleReaderFailure(error, processID: processID)
            }
        }
    }

    private func routeIncomingLine(_ rawLine: String, processID: ObjectIdentifier) {
        guard let handle, ObjectIdentifier(handle.process) == processID else {
            return
        }

        guard let responseData = rawLine.data(using: .utf8) else {
            handleReaderFailure(BackendClient.BackendClientError.invalidUTF8, processID: processID)
            return
        }

        do {
            var response = try JSONDecoder().decode(BackendResponse.self, from: responseData)
            response.rawJSON = Self.retainedRawJSON(for: response.action, rawLine: rawLine)
            if let requestID = response.request_id, let continuation = pendingResponses.removeValue(forKey: requestID) {
                continuation.resume(returning: response)
                return
            }
            eventContinuation?.yield(response)
        } catch {
            handleReaderFailure(BackendClient.BackendClientError.invalidJSON("\(error)\n\n\(rawLine)"), processID: processID)
        }
    }

    private func handleReaderFailure(_ error: Error, processID: ObjectIdentifier) {
        guard let handle, ObjectIdentifier(handle.process) == processID else {
            return
        }
        self.handle = nil
        readerTask?.cancel()
        readerTask = nil
        streamSubscriptionKey = nil
        failAllPending(with: error)
        eventContinuation?.finish()
        eventContinuation = nil
    }

    private static func readLine(stdout: FileHandle, stderr: FileHandle, process: Process) throws -> String {
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

    private static func readStderr(_ stderr: FileHandle) throws -> String {
        let data = try stderr.readToEnd() ?? Data()
        return String(data: data, encoding: .utf8) ?? ""
    }

    private static func retainedRawJSON(for action: String, rawLine: String) -> String {
        if action.hasPrefix("stream-") {
            return ""
        }
        if rawLine.count <= maxRetainedRawJSONCharacters {
            return rawLine
        }
        let keep = rawLine.prefix(maxRetainedRawJSONCharacters)
        return "\(keep)\n\n... raw JSON truncated ..."
    }
}
