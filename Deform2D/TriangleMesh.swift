
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
        initialize(vertexSizeHint: vertexSizeHint, triangleSizeHint: triangleSizeHint)
    }

    private func initialize(vertexSizeHint: Int, triangleSizeHint: Int) {
        if vertexSizeHint > 0 {
            vertices.reserveCapacity(vertexSizeHint)
            normals.reserveCapacity(vertexSizeHint)
            colors.reserveCapacity(vertexSizeHint)
            textureCoords.reserveCapacity(vertexSizeHint)
        }
        if triangleSizeHint > 0 {
            triangles.reserveCapacity(triangleSizeHint * 3)
        }
    }
    
    func getVertex(vertexIndex: VertexIndex, vertex: inout SIMD3<Float>) {
        vertex = vertices[vertexIndex]
    }

    func getVertex(vertexIndex: VertexIndex, vertex: inout SIMD3<Float>, normal: inout SIMD3<Float>?) {
        vertex = vertices[vertexIndex]
        if normal != nil && vertexIndex < normals.count {
            normal! = normals[vertexIndex]
        }
    }

    func getTriangle(triangleIndex: TriangleIndex, vTriangle: inout [SIMD3<Float>], pNormals: inout [SIMD3<Float>]?) {
        let triIndices = getTriangleIndices(triangleIndex: triangleIndex)
        vTriangle[0] = vertices[Int(triIndices[0])]
        vTriangle[1] = vertices[Int(triIndices[1])]
        vTriangle[2] = vertices[Int(triIndices[2])]
        if pNormals != nil {
            pNormals![0] = normals[Int(triIndices[0])]
            pNormals![1] = normals[Int(triIndices[1])]
            pNormals![2] = normals[Int(triIndices[2])]
        }
    }
    
    func getTriangle(i: Int, v: inout [VertexIndex]) {
        let startIndex = i * 3
        v[0] = triangles[startIndex]
        v[1] = triangles[startIndex + 1]
        v[2] = triangles[startIndex + 2]
    }

    func getNumVertices() -> Int {
        return vertices.count
    }

    func getNumTriangles() -> Int {
        return triangles.count / 3
    }

    func setVertex(i: Int, v: SIMD3<Float>) {
        if i < vertices.count {
            vertices[i] = v
        }
    }

    private func getTriangleIndices(triangleIndex: TriangleIndex) -> [VertexIndex] {
        let startIndex = triangleIndex * 3
        return [triangles[startIndex], triangles[startIndex + 1], triangles[startIndex + 2]]
    }
}
