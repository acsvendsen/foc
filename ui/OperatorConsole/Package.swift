// swift-tools-version: 6.1
import PackageDescription

let package = Package(
    name: "OperatorConsole",
    platforms: [
        .macOS(.v13),
    ],
    products: [
        .executable(name: "OperatorConsoleApp", targets: ["OperatorConsoleApp"]),
    ],
    targets: [
        .executableTarget(
            name: "OperatorConsoleApp",
            path: "Sources/OperatorConsoleApp"
        ),
    ]
)
