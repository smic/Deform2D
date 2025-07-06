
import Foundation
import simd

public final class TriangleMesh {
    public typealias VertexIndex = Int
    public typealias TriangleIndex = Int

    private var vertices: [Vector2f] = []
    private var triangles: [VertexIndex] = []

    public init(vertexSizeHint: Int = 0, triangleSizeHint: Int = 0) {
        self.initialize(vertexSizeHint: vertexSizeHint, triangleSizeHint: triangleSizeHint)
    }
    
    public func flipVertical() {
        for i in 0 ..< self.vertices.count {
            var vertex = self.vertices[i]
            vertex.y = -vertex.y
            self.vertices[i] = vertex
        }
    }

    private func initialize(vertexSizeHint: Int, triangleSizeHint: Int) {
        if vertexSizeHint > 0 {
            self.vertices.reserveCapacity(vertexSizeHint)
        }
        if triangleSizeHint > 0 {
            self.triangles.reserveCapacity(triangleSizeHint * 3)
        }
    }
    
    func getVertex(vertexIndex: VertexIndex, vertex: inout Vector2f) {
        vertex = self.vertices[vertexIndex]
    }

    func getTriangle(triangleIndex: TriangleIndex, vertices: inout [Vector2f]) {
        let triangleIndices = self.getTriangleIndices(triangleIndex: triangleIndex)
        vertices[0] = self.vertices[triangleIndices[0]]
        vertices[1] = self.vertices[triangleIndices[1]]
        vertices[2] = self.vertices[triangleIndices[2]]
    }
    
    func getTriangle(triangleIndex: TriangleIndex, vertexIndices: inout [VertexIndex]) {
        let startIndex = triangleIndex * 3
        vertexIndices[0] = self.triangles[startIndex]
        vertexIndices[1] = self.triangles[startIndex + 1]
        vertexIndices[2] = self.triangles[startIndex + 2]
    }

    func getVerticesCount() -> Int {
        return self.vertices.count
    }

    func getTrianglesCount() -> Int {
        return self.triangles.count / 3
    }

    @discardableResult
    func appendTriangle(v1: VertexIndex, v2: VertexIndex, v3: VertexIndex) -> TriangleIndex {
        let newTriangleIndex = self.triangles.count / 3
        self.triangles.append(v1)
        self.triangles.append(v2)
        self.triangles.append(v3)
        return TriangleIndex(newTriangleIndex)
    }

    @discardableResult
    func appendVertex(vertex: Vector2f) -> VertexIndex {
        let newID = self.vertices.count
        self.vertices.append(vertex)
        return VertexIndex(newID)
    }

    func setVertex(vertexIndex: VertexIndex, vertex: Vector2f) {
        if vertexIndex < self.vertices.count {
            self.vertices[vertexIndex] = vertex
        }
    }

    convenience init(fileURL: URL) throws {
        self.init()
        try self.read(from: fileURL)
    }

    func read(from url: URL) throws {
        let content = try String(contentsOf: url, encoding: .utf8)

        let lines = content.split(separator: "\n")
//        var tempNormals: [SIMD3<Float>] = []

        for line in lines {
            let components = line.split(separator: " ")
            guard let command = components.first else { continue }

            switch command {
            case "v":
                guard components.count >= 4,
                      let x = Float(components[1]),
                      let y = Float(components[2]),
                      let _ = Float(components[3]) else { continue }
                self.appendVertex(vertex: Vector2f(x, y))
            case "vn":
//                guard components.count >= 4,
//                      let x = Float(components[1]),
//                      let y = Float(components[2]),
//                      let z = Float(components[3]) else { continue }
//                tempNormals.append(normalize(SIMD3<Float>(x, y, z)))
                break
            case "f":
                guard components.count >= 4 else { continue }
                var vertexIndices: [VertexIndex] = []
                for i in 1 ..< 4 {
                    let faceComponents = components[i].split(separator: "/")
                    if let vIndex = Int(faceComponents[0]) {
                        vertexIndices.append(vIndex - 1)
                    }
                }
                if vertexIndices.count == 3 {
                    self.appendTriangle(v1: vertexIndices[0], v2: vertexIndices[1], v3: vertexIndices[2])
                }
            default:
                break
            }
        }
    }

    private func getTriangleIndices(triangleIndex: TriangleIndex) -> [VertexIndex] {
        let startIndex = triangleIndex * 3
        return [
            self.triangles[startIndex],
            self.triangles[startIndex + 1],
            self.triangles[startIndex + 2]
        ]
    }
}
