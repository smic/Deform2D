
import Foundation
import simd

@Observable
final class DeformationViewModel {
    @ObservationIgnored
    var deformedMesh: TriangleMesh
    var selectedVertices: Set<TriangleMesh.VertexIndex> = []

    @ObservationIgnored
    private var deformer = RigidMeshDeformer2D()
    @ObservationIgnored
    private var initialMesh: TriangleMesh
    @ObservationIgnored
    private var constraintsValid: Bool = false
    private var selectedVertex: TriangleMesh.VertexIndex? = nil

    init() {
        if let url = Bundle.main.url(forResource: "man", withExtension: "obj"),
           let mesh = TriangleMesh(fileURL: url) {
            mesh.flipVertical()
            self.initialMesh = mesh
            self.deformedMesh = mesh
        } else {
            let mesh = TriangleMesh()
            // Manually create a square mesh for now
            let nRowLen: UInt32 = 5
            let fYStep: Float = 2.0 / Float(nRowLen - 1)
            let fXStep: Float = 2.0 / Float(nRowLen - 1)

            for yi in 0 ..< nRowLen {
                let fY = -1.0 + Float(yi) * fYStep
                for xi in 0 ..< nRowLen {
                    let fX = -1.0 + Float(xi) * fXStep
                    mesh.appendVertex(vertex: SIMD3<Float>(fX, fY, 0))
                }
            }

            for yi in 0 ..< (nRowLen - 1) {
                let nRow1 = yi * nRowLen
                let nRow2 = (yi + 1) * nRowLen

                for xi in 0 ..< (nRowLen - 1) {
                    let nTri1: [UInt32] = [nRow1 + xi, nRow2 + xi + 1, nRow1 + xi + 1]
                    let nTri2: [UInt32] = [nRow1 + xi, nRow2 + xi, nRow2 + xi + 1]
                    mesh.appendTriangle(v1: Int(nTri1[0]), v2: Int(nTri1[1]), v3: Int(nTri1[2]))
                    mesh.appendTriangle(v1: Int(nTri2[0]), v2: Int(nTri2[1]), v3: Int(nTri2[2]))
                }
            }
            self.initialMesh = mesh
            self.deformedMesh = mesh
        }
        
        self.deformer.initializeFromMesh(mesh: self.initialMesh)
        self.invalidateConstraints()
    }

    func updateDeformedMesh() {
        self.validateConstraints()
        self.deformer.updateDeformedMesh(mesh: &self.deformedMesh, rigid: true)
    }

    func invalidateConstraints() {
        self.constraintsValid = false
    }

    func validateConstraints() {
        if self.constraintsValid {
            return
        }

        for vertex in self.selectedVertices {
            var v = SIMD3<Float>()
            var n: SIMD3<Float>?
            self.deformedMesh.getVertex(vertexIndex: vertex, vertex: &v, normal: &n)
            self.deformer.setDeformedHandle(handle: vertex, pos: SIMD2<Float>(v.x, v.y))
        }

        self.deformer.forceValidation()
        self.constraintsValid = true
    }

    func findHitVertex(point: CGPoint, size: CGSize) -> Int? {
        let scale = 0.5 * min(size.width, size.height) / 2.0
        let translate = SIMD2<Float>(Float(size.width / 2), Float(size.height / 2))

        for i in 0 ..< self.deformedMesh.getNumVertices() {
            var v = SIMD3<Float>()
            var n: SIMD3<Float>?
            self.deformedMesh.getVertex(vertexIndex: i, vertex: &v, normal: &n)
            let viewPos = SIMD2<Float>(v.x, v.y) * Float(scale) + translate
            let dist = distance(SIMD2<Float>(Float(point.x), Float(point.y)), viewPos)
            if dist < 5 {
                return i
            }
        }
        return nil
    }

    func handleDrag(point: CGPoint, size: CGSize) {
        print("handleDrag:", point, "selectedVertex:", selectedVertex)
        if let selectedVertex = self.selectedVertex {
            let scale = 0.5 * min(size.width, size.height) / 2.0
            let translate = SIMD2<Float>(Float(size.width / 2), Float(size.height / 2))
            let worldPos = (SIMD2<Float>(Float(point.x), Float(point.y)) - translate) / Float(scale)
            print("deform:", selectedVertex, "x:", worldPos.x, "y:", worldPos.y)
            self.deformedMesh.setVertex(i: selectedVertex, v: SIMD3<Float>(worldPos.x, worldPos.y, 0))
            self.invalidateConstraints()
        }
    }

    func selectVertex(point: CGPoint, size: CGSize) {
        print("selectVertex:", self.findHitVertex(point: point, size: size))
        if let hit = findHitVertex(point: point, size: size) {
            self.selectedVertex = hit
        }
    }

    func toggleSelection(point: CGPoint, size: CGSize) {
        print("toggle Selection:", self.findHitVertex(point: point, size: size))
        if let hit = findHitVertex(point: point, size: size) {
            if self.selectedVertices.contains(hit) {
                self.selectedVertices.remove(hit)
                self.deformer.removeHandle(handle: hit)
                var v = SIMD3<Float>()
                var n: SIMD3<Float>?
                self.initialMesh.getVertex(vertexIndex: hit, vertex: &v, normal: &n)
                self.deformedMesh.setVertex(i: Int(hit), v: v)
            } else {
                self.selectedVertices.insert(hit)
            }
            self.invalidateConstraints()
        }
    }
    
    func releaseSelection() {
        self.selectedVertex = nil
    }
}
