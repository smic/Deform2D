
import Foundation
import simd

@Observable
final class DeformationViewModel {
    var deformedMesh: TriangleMesh
    var selectedVertices: Set<TriangleMesh.VertexIndex> = []

    @ObservationIgnored
    private var deformer = RigidMeshDeformer2D()
    @ObservationIgnored
    private var initialMesh: TriangleMesh
    @ObservationIgnored
    private var constraintsValid: Bool = false
    @ObservationIgnored
    private var selectedVertex: TriangleMesh.VertexIndex? = nil

    init() {
        let mesh = TriangleMesh()
        // Manually create a square mesh for now
        let nRowLen: Int = 5
        let fYStep: Float = 2.0 / Float(nRowLen - 1)
        let fXStep: Float = 2.0 / Float(nRowLen - 1)

        for yi in 0..<nRowLen {
            let fY = -1.0 + Float(yi) * fYStep
            for xi in 0..<nRowLen {
                let fX = -1.0 + Float(xi) * fXStep
                mesh.appendVertex(vertex: SIMD3<Float>(fX, fY, 0))
            }
        }

        for yi in 0..<(nRowLen - 1) {
            let nRow1 = yi * nRowLen
            let nRow2 = (yi + 1) * nRowLen

            for xi in 0..<(nRowLen - 1) {
                let nTri1: [TriangleMesh.VertexIndex] = [nRow1 + xi, nRow2 + xi + 1, nRow1 + xi + 1]
                let nTri2: [TriangleMesh.VertexIndex] = [nRow1 + xi, nRow2 + xi, nRow2 + xi + 1]
                mesh.appendTriangle(v1: nTri1[0], v2: nTri1[1], v3: nTri1[2])
                mesh.appendTriangle(v1: nTri2[0], v2: nTri2[1], v3: nTri2[2])
            }
        }
        
        self.initialMesh = mesh
        self.deformedMesh = mesh
        self.deformer.initializeFromMesh(mesh: initialMesh)
        invalidateConstraints()
    }

    func updateDeformedMesh() {
        validateConstraints()
        deformer.updateDeformedMesh(mesh: &deformedMesh, rigid: true)
    }

    func invalidateConstraints() {
        constraintsValid = false
    }

    func validateConstraints() {
        if constraintsValid {
            return
        }

        for vertex in selectedVertices {
            var v = SIMD3<Float>()
            var n: SIMD3<Float>?
            deformedMesh.getVertex(vertexIndex: vertex, vertex: &v, normal: &n)
            deformer.setDeformedHandle(handle: vertex, pos: SIMD2<Float>(v.x, v.y))
        }

        deformer.forceValidation()
        constraintsValid = true
    }

    func findHitVertex(point: CGPoint, size: CGSize) -> Int? {
        let scale = 0.5 * min(size.width, size.height) / 2.0
        let translate = SIMD2<Float>(Float(size.width / 2), Float(size.height / 2))

        for i in 0..<deformedMesh.getNumVertices() {
            var v = SIMD3<Float>()
            var n: SIMD3<Float>?
            deformedMesh.getVertex(vertexIndex: i, vertex: &v, normal: &n)
            let viewPos = SIMD2<Float>(v.x, v.y) * Float(scale) + translate
            let dist = distance(SIMD2<Float>(Float(point.x), Float(point.y)), viewPos)
            if dist < 5 {
                return i
            }
        }
        return nil
    }

    func handleDrag(point: CGPoint, size: CGSize) {
        if let selectedVertex = selectedVertex {
            let scale = 0.5 * min(size.width, size.height) / 2.0
            let translate = SIMD2<Float>(Float(size.width / 2), Float(size.height / 2))
            let worldPos = (SIMD2<Float>(Float(point.x), Float(point.y)) - translate) / Float(scale)
            deformedMesh.setVertex(i: Int(selectedVertex), v: SIMD3<Float>(worldPos.x, worldPos.y, 0))
            invalidateConstraints()
        }
    }

    func selectVertex(point: CGPoint, size: CGSize) {
        if let hit = findHitVertex(point: point, size: size) {
            selectedVertex = hit
        }
    }

    func toggleSelection(point: CGPoint, size: CGSize) {
        if let hit = findHitVertex(point: point, size: size) {
            if selectedVertices.contains(hit) {
                selectedVertices.remove(hit)
                deformer.removeHandle(handle: hit)
                var v = SIMD3<Float>()
                var n: SIMD3<Float>?
                initialMesh.getVertex(vertexIndex: hit, vertex: &v, normal: &n)
                deformedMesh.setVertex(i: Int(hit), v: v)
            } else {
                selectedVertices.insert(hit)
            }
            invalidateConstraints()
        }
    }
    
    func releaseSelection() {
        selectedVertex = nil
    }
}
