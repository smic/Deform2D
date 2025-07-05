
import Foundation
import simd

class TriangleMesh {
    typealias VertexIndex = Int
    typealias TriangleIndex = Int

    private var vertices: [SIMD3<Float>] = []
    private var normals: [SIMD3<Float>] = []
    private var colors: [SIMD3<Float>] = []
    private var textureCoords: [SIMD2<Float>] = []
    private var triangles: [VertexIndex] = []
    private var triTexCoords: [SIMD2<Float>] = []

    enum FileFormat {
        case objFormat, meshLiteFormat
    }

    private var filename: String = "default.obj"
    private var fileFormat: FileFormat = .objFormat
    private var fileError: String = "no error"

    init(filename: String = "default.obj", vertexSizeHint: Int = 0, triangleSizeHint: Int = 0) {
        self.filename = filename
        self.initialize(vertexSizeHint: vertexSizeHint, triangleSizeHint: triangleSizeHint)
    }

    private func initialize(vertexSizeHint: Int, triangleSizeHint: Int) {
        if vertexSizeHint > 0 {
            self.vertices.reserveCapacity(vertexSizeHint)
            self.normals.reserveCapacity(vertexSizeHint)
            self.colors.reserveCapacity(vertexSizeHint)
            self.textureCoords.reserveCapacity(vertexSizeHint)
        }
        if triangleSizeHint > 0 {
            self.triangles.reserveCapacity(triangleSizeHint * 3)
        }
    }
    
    func getVertex(vertexIndex: VertexIndex, vertex: inout SIMD3<Float>) {
        vertex = self.vertices[vertexIndex]
    }

    func getVertex(vertexIndex: VertexIndex, vertex: inout SIMD3<Float>, normal: inout SIMD3<Float>?) {
        vertex = self.vertices[vertexIndex]
        if normal != nil && vertexIndex < self.normals.count {
            normal! = self.normals[vertexIndex]
        }
    }

    func getTriangle(triangleIndex: TriangleIndex, vTriangle: inout [SIMD3<Float>], pNormals: inout [SIMD3<Float>]?) {
        let triIndices = self.getTriangleIndices(triangleIndex: triangleIndex)
        vTriangle[0] = self.vertices[triIndices[0]]
        vTriangle[1] = self.vertices[triIndices[1]]
        vTriangle[2] = self.vertices[triIndices[2]]
        if pNormals != nil {
            pNormals![0] = self.normals[triIndices[0]]
            pNormals![1] = self.normals[triIndices[1]]
            pNormals![2] = self.normals[triIndices[2]]
        }
    }
    
    func getTriangle(i: Int, v: inout [VertexIndex]) {
        let startIndex = i * 3
        v[0] = self.triangles[startIndex]
        v[1] = self.triangles[startIndex + 1]
        v[2] = self.triangles[startIndex + 2]
    }

    func getNumVertices() -> Int {
        return self.vertices.count
    }

    func getNumTriangles() -> Int {
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
    func appendVertex(vertex: SIMD3<Float>) -> VertexIndex {
        let newID = self.vertices.count
        self.vertices.append(vertex)
        return VertexIndex(newID)
    }

    func setVertex(i: VertexIndex, v: SIMD3<Float>) {
        if i < self.vertices.count {
            self.vertices[i] = v
        }
    }

    private func getTriangleIndices(triangleIndex: TriangleIndex) -> [VertexIndex] {
        let startIndex = triangleIndex * 3
        return [self.triangles[startIndex], self.triangles[startIndex + 1], self.triangles[startIndex + 2]]
    }
}
