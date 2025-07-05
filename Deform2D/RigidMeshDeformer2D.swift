
import Foundation
import simd
import Accelerate

typealias Vector2f = SIMD2<Float>
typealias Vector3f = SIMD3<Float>
typealias Matrix = simd_float4x4

class RigidMeshDeformer2D {
    
    struct Vertex {
        var position: Vector2f
    }
    
    struct Triangle {
        var verts: [UInt32] = [0,0,0]
        var triCoords: [Vector2f] = [Vector2f.zero, Vector2f.zero, Vector2f.zero]
        var scaled: [Vector2f] = [Vector2f.zero, Vector2f.zero, Vector2f.zero]
        var f: Matrix = Matrix()
        var c: Matrix = Matrix()
    }
    
    struct Constraint: Comparable, Hashable {
        var vertex: UInt32
        var constrainedPos: Vector2f
        
        static func < (lhs: RigidMeshDeformer2D.Constraint, rhs: RigidMeshDeformer2D.Constraint) -> Bool {
            return lhs.vertex < rhs.vertex
        }
    }
    
    private var initialVerts: [Vertex] = []
    private var deformedVerts: [Vertex] = []
    private var triangles: [Triangle] = []
    private var constraints: Set<Constraint> = []
    private var setupValid: Bool = false
    
    private var firstMatrix: [Double] = []
    private var vertexMap: [Int] = []
    private var hxPrime: Matrix = Matrix()
    private var hyPrime: Matrix = Matrix()
    private var dx: Matrix = Matrix()
    private var dy: Matrix = Matrix()
    
    // These would need a proper implementation of LUDecomposition
    // For now, just placeholder types
    private var luDecompX: Any?
    private var luDecompY: Any?
    
    
    init() {
        invalidateSetup()
    }
    
    func forceValidation() {
        validateSetup()
    }
    
    func removeHandle(handle: UInt32) {
        let c = Constraint(vertex: handle, constrainedPos: Vector2f.zero)
        constraints.remove(c)
        deformedVerts[Int(handle)].position = initialVerts[Int(handle)].position
        invalidateSetup()
    }
    
    func setDeformedHandle(handle: UInt32, pos: Vector2f) {
        let c = Constraint(vertex: handle, constrainedPos: pos)
        updateConstraint(cons: c)
    }
    
    func unTransformPoint(transform: inout Vector2f) {
        for i in 0..<triangles.count {
            let v1 = deformedVerts[Int(triangles[i].verts[0])].position
            let v2 = deformedVerts[Int(triangles[i].verts[1])].position
            let v3 = deformedVerts[Int(triangles[i].verts[2])].position
            
            let barycentric = barycentricCoords(p: transform, a: v1, b: v2, c: v3)
            if barycentric.x < 0 || barycentric.x > 1 || barycentric.y < 0 || barycentric.y > 1 || barycentric.z < 0 || barycentric.z > 1 {
                continue
            }
            
            let v1Init = initialVerts[Int(triangles[i].verts[0])].position
            let v2Init = initialVerts[Int(triangles[i].verts[1])].position
            let v3Init = initialVerts[Int(triangles[i].verts[2])].position
            transform = barycentric.x * v1Init + barycentric.y * v2Init + barycentric.z * v3Init
            return
        }
    }
    
    func initializeFromMesh(mesh: TriangleMesh) {
        constraints.removeAll()
        initialVerts.removeAll()
        deformedVerts.removeAll()
        triangles.removeAll()
        
        let nVerts = mesh.getNumVertices()
        for i in 0..<nVerts {
            var vertex: Vector3f = Vector3f.zero
            mesh.getVertex(i: i, v: &vertex)
            let v = Vertex(position: Vector2f(vertex.x, vertex.y))
            initialVerts.append(v)
            deformedVerts.append(v)
        }
        
        let nTris = mesh.getNumTriangles()
        for i in 0..<nTris {
            var t = Triangle()
            var verts: [UInt32] = [0,0,0]
            mesh.getTriangle(i: i, v: &verts)
            t.verts = verts
            triangles.append(t)
        }
        
        for i in 0..<nTris {
            var t = triangles[i]
            for j in 0..<3 {
                let n0 = j
                let n1 = (j+1)%3
                let n2 = (j+2)%3
                
                let v0 = getInitialVert(nVert: t.verts[n0])
                let v1 = getInitialVert(nVert: t.verts[n1])
                let v2 = getInitialVert(nVert: t.verts[n2])
                
                let v01 = v1 - v0
                let v01N = normalize(v01)
                let v01Rot90 = Vector2f(v01.y, -v01.x)
                
                let vLocal = v2 - v0
                let fX = dot(vLocal, v01) / dot(v01, v01)
                let fY = dot(vLocal, v01Rot90) / dot(v01Rot90, v01Rot90)
                
                t.triCoords[j] = Vector2f(fX, fY)
            }
            triangles[i] = t
        }
    }
    
    func updateDeformedMesh(mesh: inout TriangleMesh, rigid: Bool) {
        validateDeformedMesh(rigid: rigid)
        
        let useVerts = constraints.count > 1 ? deformedVerts : initialVerts
        
        let nVerts = mesh.getNumVertices()
        for i in 0..<nVerts {
            let newPos = useVerts[Int(i)].position
            mesh.setVertex(i: i, v: Vector3f(newPos.x, newPos.y, 0.0))
        }
    }
    
    private func updateConstraint(cons: Constraint) {
        if constraints.contains(cons) {
            constraints.update(with: cons)
        } else {
            constraints.insert(cons)
        }
        deformedVerts[Int(cons.vertex)].position = cons.constrainedPos
        invalidateSetup()
    }
    
    private func invalidateSetup() {
        setupValid = false
    }
    
    private func validateSetup() {
        if setupValid || constraints.count < 2 {
            return
        }
        
        precomputeOrientationMatrix()
        
        for i in 0..<triangles.count {
            precomputeScalingMatrices(nTriangle: i)
        }
        
        precomputeFittingMatrices()
        
        setupValid = true
    }
    
    private func precomputeOrientationMatrix() {
        let constraintsVec = constraints.sorted()
        let nVerts = deformedVerts.count
        let nConstraints = constraintsVec.count
        let nFreeVerts = nVerts - nConstraints

        vertexMap = [Int](repeating: 0, count: nVerts)
        var nRow = 0
        for i in 0..<nVerts {
            let c = Constraint(vertex: UInt32(i), constrainedPos: .zero)
            if !constraints.contains(c) {
                vertexMap[i] = nRow
                nRow += 1
            }
        }
        for i in 0..<nConstraints {
            vertexMap[Int(constraintsVec[i].vertex)] = nRow
            nRow += 1
        }

        let matrixSize = 2 * nVerts
        var firstMatrix = [Double](repeating: 0.0, count: matrixSize * matrixSize)

        for i in 0..<triangles.count {
            let t = triangles[i]
            for j in 0..<3 {
                let n0x = 2 * vertexMap[Int(t.verts[j])]
                let n0y = n0x + 1
                let n1x = 2 * vertexMap[Int(t.verts[(j + 1) % 3])]
                let n1y = n1x + 1
                let n2x = 2 * vertexMap[Int(t.verts[(j + 2) % 3])]
                let n2y = n2x + 1
                let x = Double(t.triCoords[j].x)
                let y = Double(t.triCoords[j].y)

                // n0x,n?? elems
                firstMatrix[n0x * matrixSize + n0x] += 1 - 2*x + x*x + y*y
                firstMatrix[n0x * matrixSize + n1x] += 2*x - 2*x*x - 2*y*y
                firstMatrix[n0x * matrixSize + n1y] += 2*y
                firstMatrix[n0x * matrixSize + n2x] += -2 + 2*x
                firstMatrix[n0x * matrixSize + n2y] += -2 * y

                // n0y,n?? elems
                firstMatrix[n0y * matrixSize + n0y] += 1 - 2*x + x*x + y*y
                firstMatrix[n0y * matrixSize + n1x] += -2*y
                firstMatrix[n0y * matrixSize + n1y] += 2*x - 2*x*x - 2*y*y
                firstMatrix[n0y * matrixSize + n2x] += 2*y
                firstMatrix[n0y * matrixSize + n2y] += -2 + 2*x

                // n1x,n?? elems
                firstMatrix[n1x * matrixSize + n1x] += x*x + y*y
                firstMatrix[n1x * matrixSize + n2x] += -2*x
                firstMatrix[n1x * matrixSize + n2y] += 2*y

                //n1y,n?? elems
                firstMatrix[n1y * matrixSize + n1y] += x*x + y*y
                firstMatrix[n1y * matrixSize + n2x] += -2*y
                firstMatrix[n1y * matrixSize + n2y] += -2*x

                // final 2 elems
                firstMatrix[n2x * matrixSize + n2x] += 1
                firstMatrix[n2y * matrixSize + n2y] += 1
            }
        }

        let freeSize = 2 * nFreeVerts
        let constSize = 2 * nConstraints

        var g00 = [Double](repeating: 0.0, count: freeSize * freeSize)
        var g01 = [Double](repeating: 0.0, count: freeSize * constSize)
        var g10 = [Double](repeating: 0.0, count: constSize * freeSize)

        for i in 0..<freeSize {
            for j in 0..<freeSize {
                g00[i * freeSize + j] = firstMatrix[i * matrixSize + j]
            }
        }

        for i in 0..<freeSize {
            for j in 0..<constSize {
                g01[i * constSize + j] = firstMatrix[i * matrixSize + (j + freeSize)]
            }
        }

        for i in 0..<constSize {
            for j in 0..<freeSize {
                g10[i * freeSize + j] = firstMatrix[(i + freeSize) * matrixSize + j]
            }
        }

        var gPrime = g00
        for i in 0..<freeSize {
            for j in 0..<freeSize {
                gPrime[i * freeSize + j] += g00[j * freeSize + i]
            }
        }

        var b = g01
        for i in 0..<freeSize {
            for j in 0..<constSize {
                b[i * constSize + j] += g10[j * freeSize + i]
            }
        }

        var ipiv = [__CLPK_integer](repeating: 0, count: freeSize)
        var lwork = __CLPK_integer(freeSize * freeSize)
        var work = [__CLPK_doublereal](repeating: 0, count: Int(lwork))
        var error: __CLPK_integer = 0
        var n = __CLPK_integer(freeSize)

        dgetrf_(&n, &n, &gPrime, &n, &ipiv, &error)
        if error != 0 {
            print("Error in LU factorization")
            return
        }

        dgetri_(&n, &gPrime, &n, &ipiv, &work, &lwork, &error)
        if error != 0 {
            print("Error in matrix inversion")
            return
        }

        var finalMatrix = [Double](repeating: 0.0, count: freeSize * constSize)
        let alpha = -1.0
        let beta = 0.0
        var m = __CLPK_integer(freeSize)
        var k = __CLPK_integer(constSize)
        n = __CLPK_integer(freeSize)


        dgemm_("N".cString(using: .utf8)!, "N".cString(using: .utf8)!, &m, &k, &n, &alpha, &gPrime, &m, &b, &n, &beta, &finalMatrix, &m)

        self.firstMatrix = finalMatrix
    }
    
    private func precomputeScalingMatrices(nTriangle: Int) {
        // This is a complex matrix setup that would require a full
        // port of the Wml::GMatrixd and related linear algebra code.
        // For now, this is a placeholder.
    }
    
    private func precomputeFittingMatrices() {
        // This is a complex matrix setup that would require a full
        // port of the Wml::GMatrixd and related linear algebra code.
        // For now, this is a placeholder.
    }
    
    private func validateDeformedMesh(rigid: Bool) {
        if constraints.count < 2 {
            return
        }
        
        validateSetup()
        
        // This is a complex matrix setup that would require a full
        // port of the Wml::GMatrixd and related linear algebra code.
        // For now, this is a placeholder.
        
        if rigid {
            for i in 0..<triangles.count {
                updateScaledTriangle(nTriangle: i)
            }
            applyFittingStep()
        }
    }
    
    private func updateScaledTriangle(nTriangle: Int) {
        // This is a complex matrix setup that would require a full
        // port of the Wml::GMatrixd and related linear algebra code.
        // For now, this is a placeholder.
    }
    
    private func applyFittingStep() {
        // This is a complex matrix setup that would require a full
        // port of the Wml::GMatrixd and related linear algebra code.
        // For now, this is a placeholder.
    }
    
    private func getInitialVert(nVert: UInt32) -> Vector2f {
        return initialVerts[Int(nVert)].position
    }
    
    private func barycentricCoords(p: Vector2f, a: Vector2f, b: Vector2f, c: Vector2f) -> Vector3f {
        let v0 = b - a
        let v1 = c - a
        let v2 = p - a
        let d00 = dot(v0, v0)
        let d01 = dot(v0, v1)
        let d11 = dot(v1, v1)
        let d20 = dot(v2, v0)
        let d21 = dot(v2, v1)
        let denom = d00 * d11 - d01 * d01
        let v = (d11 * d20 - d01 * d21) / denom
        let w = (d00 * d21 - d01 * d20) / denom
        let u = 1.0 - v - w
        return Vector3f(u, v, w)
    }
}

// Dummy TriangleMesh for compilation
class TriangleMesh {
    func getNumVertices() -> Int { return 0 }
    func getNumTriangles() -> Int { return 0 }
    func getVertex(i: Int, v: inout Vector3f) {}
    func getTriangle(i: Int, v: inout [UInt32]) {}
    func setVertex(i: Int, v: Vector3f) {}
}
