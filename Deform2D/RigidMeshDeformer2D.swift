
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
        var f: [Double] = []
        var c: [Double] = []
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
    
    struct LUDecomposition {
        var luMatrix: [Double]
        var pivotIndices: [__LAPACK_int]
    }

    private var firstMatrix: [Double] = []
    private var vertexMap: [Int] = []
    private var hxPrime: [Double] = []
    private var hyPrime: [Double] = []
    private var dx: [Double] = []
    private var dy: [Double] = []
    
    private var luDecompX: LUDecomposition?
    private var luDecompY: LUDecomposition?
    
    
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

        var ipiv = [__LAPACK_int](repeating: 0, count: freeSize)
        var lwork = __LAPACK_int(freeSize * freeSize)
        var work = [Double](repeating: 0, count: Int(lwork))
        var error: __LAPACK_int = 0
        var n_lapack = __LAPACK_int(freeSize)

        dgetrf_(&n_lapack, &n_lapack, &gPrime, &n_lapack, &ipiv, &error)
        if error != 0 {
            print("Error in LU factorization")
            return
        }

        dgetri_(&n_lapack, &gPrime, &n_lapack, &ipiv, &work, &lwork, &error)
        if error != 0 {
            print("Error in matrix inversion")
            return
        }

        var finalMatrix = [Double](repeating: 0.0, count: freeSize * constSize)
        let alpha = -1.0
        let beta = 0.0
        let M = Int32(freeSize)
        let N = Int32(constSize)
        let K = Int32(freeSize)

        cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, M, N, K, alpha, &gPrime, K, &b, N, beta, &finalMatrix, N)

        self.firstMatrix = finalMatrix
    }
    
    private func precomputeScalingMatrices(nTriangle: Int) {
        var t = triangles[nTriangle]

        t.f = [Double](repeating: 0.0, count: 16)
        t.c = [Double](repeating: 0.0, count: 24)

        let x01 = Double(t.triCoords[0].x)
        let y01 = Double(t.triCoords[0].y)
        let x12 = Double(t.triCoords[1].x)
        let y12 = Double(t.triCoords[1].y)
        let x20 = Double(t.triCoords[2].x)
        let y20 = Double(t.triCoords[2].y)

        let k1 = x12*y01 + (-1 + x01)*y12
        let k2 = -x12 + x01*x12 - y01*y12
        let k3 = -y01 + x20*y01 + x01*y20
        let k4 = -y01 + x01*y01 + x01*y20
        let k5 = -x01 + x01*x20 - y01*y20

        let a = -1 + x01
        let a1 = pow(-1 + x01,2) + pow(y01,2)
        let a2 = pow(x01,2) + pow(y01,2)
        let b =  -1 + x20
        let b1 = pow(-1 + x20,2) + pow(y20,2)
        let c2 = pow(x12,2) + pow(y12,2)

        let r1 = 1 + 2*a*x12 + a1*pow(x12,2) - 2*y01*y12 + a1*pow(y12,2)
        let r2 = -(b*x01) - b1*pow(x01,2) + y01*(-(b1*y01) + y20)
        let r3 = -(a*x12) - a1*pow(x12,2) + y12*(y01 - a1*y12)
        let r5 = a*x01 + pow(y01,2)
        let r6 = -(b*y01) - x01*y20
        let r7 = 1 + 2*b*x01 + b1*pow(x01,2) + b1*pow(y01,2) - 2*y01*y20

        //  set up F matrix
        t.f[0] = 2*a1 + 2*a1*c2 + 2*r7
        t.f[1] = 0
        t.f[2] = 2*r2 + 2*r3 - 2*r5
        t.f[3] = 2*k1 + 2*r6 + 2*y01

        t.f[4] = 0
        t.f[5] = 2*a1 + 2*a1*c2 + 2*r7
        t.f[6] = -2*k1 + 2*k3 - 2*y01
        t.f[7] = 2*r2 + 2*r3 - 2*r5

        t.f[8] = 2*r2 + 2*r3 - 2*r5
        t.f[9] = -2*k1 + 2*k3 - 2*y01
        t.f[10] = 2*a2 + 2*a2*b1 + 2*r1
        t.f[11] = 0

        t.f[12] = 2*k1 - 2*k3 + 2*y01
        t.f[13] = 2*r2 + 2*r3 - 2*r5
        t.f[14] = 0
        t.f[15] = 2*a2 + 2*a2*b1 + 2*r1

        var ipiv = [__LAPACK_int](repeating: 0, count: 4)
        var lwork = __LAPACK_int(16)
        var work = [Double](repeating: 0, count: Int(lwork))
        var error: __LAPACK_int = 0
        var n_lapack = __LAPACK_int(4)

        dgetrf_(&n_lapack, &n_lapack, &t.f, &n_lapack, &ipiv, &error)
        dgetri_(&n_lapack, &t.f, &n_lapack, &ipiv, &work, &lwork, &error)

        vDSP_vsmulD(t.f, 1, [-1.0], &t.f, 1, 16)

        // set up C matrix
        t.c[0] = 2*k2
        t.c[1] = -2*k1
        t.c[2] = 2*(-1-k5)
        t.c[3] = 2*k3
        t.c[4] = 2*a
        t.c[5] = -2*y01

        t.c[6] = 2*k1
        t.c[7] = 2*k2
        t.c[8] = -2*k3
        t.c[9] = 2*(-1-k5)
        t.c[10] = 2*y01
        t.c[11] = 2*a

        t.c[12] = 2*(-1-k2)
        t.c[13] = 2*k1
        t.c[14] = 2*k5
        t.c[15] = 2*r6
        t.c[16] = -2*x01
        t.c[17] = 2*y01

        t.c[18] = 2*k1
        t.c[19] = 2*(-1-k2)
        t.c[20] = -2*k3
        t.c[21] = 2*k5
        t.c[22] = -2*y01
        t.c[23] = -2*x01

        triangles[nTriangle] = t
    }
    
    private func precomputeFittingMatrices() {
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

        var hX = [Double](repeating: 0.0, count: nVerts * nVerts)
        var hY = [Double](repeating: 0.0, count: nVerts * nVerts)

        for i in 0..<triangles.count {
            let t = triangles[i]
            for j in 0..<3 {
                let nA = vertexMap[Int(t.verts[j])]
                let nB = vertexMap[Int(t.verts[(j + 1) % 3])]

                hX[nA * nVerts + nA] += 2
                hX[nA * nVerts + nB] += -2
                hX[nB * nVerts + nA] += -2
                hX[nB * nVerts + nB] += 2

                hY[nA * nVerts + nA] += 2
                hY[nA * nVerts + nB] += -2
                hY[nB * nVerts + nA] += -2
                hY[nB * nVerts + nB] += 2
            }
        }

        var hX00 = [Double](repeating: 0.0, count: nFreeVerts * nFreeVerts)
        var hY00 = [Double](repeating: 0.0, count: nFreeVerts * nFreeVerts)
        for i in 0..<nFreeVerts {
            for j in 0..<nFreeVerts {
                hX00[i * nFreeVerts + j] = hX[i * nVerts + j]
                hY00[i * nFreeVerts + j] = hY[i * nVerts + j]
            }
        }

        var hX01 = [Double](repeating: 0.0, count: nFreeVerts * nConstraints)
        var hY01 = [Double](repeating: 0.0, count: nFreeVerts * nConstraints)
        for i in 0..<nFreeVerts {
            for j in 0..<nConstraints {
                hX01[i * nConstraints + j] = hX[i * nVerts + (j + nFreeVerts)]
                hY01[i * nConstraints + j] = hY[i * nVerts + (j + nFreeVerts)]
            }
        }

        hxPrime = hX00
        hyPrime = hY00
        dx = hX01
        dy = hY01

        var ipivX = [__LAPACK_int](repeating: 0, count: nFreeVerts)
        var errorX: __LAPACK_int = 0
        var n_lapackX = __LAPACK_int(nFreeVerts)
        dgetrf_(&n_lapackX, &n_lapackX, &hxPrime, &n_lapackX, &ipivX, &errorX)
        if errorX == 0 {
            luDecompX = LUDecomposition(luMatrix: hxPrime, pivotIndices: ipivX)
        }

        var ipivY = [__LAPACK_int](repeating: 0, count: nFreeVerts)
        var errorY: __LAPACK_int = 0
        var n_lapackY = __LAPACK_int(nFreeVerts)
        dgetrf_(&n_lapackY, &n_lapackY, &hyPrime, &n_lapackY, &ipivY, &errorY)
        if errorY == 0 {
            luDecompY = LUDecomposition(luMatrix: hyPrime, pivotIndices: ipivY)
        }
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
        var t = triangles[nTriangle]

        let vDeformedV0 = deformedVerts[Int(t.verts[0])].position
        let vDeformedV1 = deformedVerts[Int(t.verts[1])].position
        let vDeformedV2 = deformedVerts[Int(t.verts[2])].position
        let vDeformed: [Double] = [Double(vDeformedV0.x), Double(vDeformedV0.y),
                                   Double(vDeformedV1.x), Double(vDeformedV1.y),
                                   Double(vDeformedV2.x), Double(vDeformedV2.y)]

        var mCVec = [Double](repeating: 0.0, count: 4)
        cblas_dgemv(CblasRowMajor, CblasNoTrans, 4, 6, 1.0, t.c, 6, vDeformed, 1, 0.0, &mCVec, 1)

        var vSolution = [Double](repeating: 0.0, count: 4)
        cblas_dgemv(CblasRowMajor, CblasNoTrans, 4, 4, 1.0, t.f, 4, mCVec, 1, 0.0, &vSolution, 1)

        var vFitted0 = Vector2f(Float(vSolution[0]), Float(vSolution[1]))
        var vFitted1 = Vector2f(Float(vSolution[2]), Float(vSolution[3]))

        let x01 = t.triCoords[0].x
        let y01 = t.triCoords[0].y
        let vFitted01 = vFitted1 - vFitted0
        let vFitted01Perp = Vector2f(vFitted01.y, -vFitted01.x)
        var vFitted2 = vFitted0 + x01 * vFitted01 + y01 * vFitted01Perp

        let vOrigV0 = initialVerts[Int(t.verts[0])].position
        let vOrigV1 = initialVerts[Int(t.verts[1])].position
        let fScale = length(vOrigV1 - vOrigV0) / length(vFitted01)

        vFitted0 *= fScale
        vFitted1 *= fScale
        vFitted2 *= fScale

        t.scaled[0] = vFitted0
        t.scaled[1] = vFitted1
        t.scaled[2] = vFitted2
        
        triangles[nTriangle] = t
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
