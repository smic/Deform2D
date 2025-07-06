
import Foundation
import simd

@Observable
public final class DeformationViewModel {
    @ObservationIgnored
    public var deformedMesh: TriangleMesh
    public var selectedVertices: Set<TriangleMesh.VertexIndex> = []
    public var selectedVertex: TriangleMesh.VertexIndex? = nil

    @ObservationIgnored
    private var deformer = RigidMeshDeformer2D()
    @ObservationIgnored
    private var initialMesh: TriangleMesh
    @ObservationIgnored
    private var constraintsValid: Bool = false

    public init() {
        if let url = Bundle.main.url(forResource: "man", withExtension: "obj"),
           let mesh = try? TriangleMesh(fileURL: url) {
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
                    mesh.appendVertex(vertex: Vector2f(fX, fY))
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

    public func updateDeformedMesh(rigid: Bool) {
        self.validateConstraints()
        self.deformer.updateDeformedMesh(mesh: &self.deformedMesh, rigid: rigid)
    }

    private func invalidateConstraints() {
        self.constraintsValid = false
    }

    private func validateConstraints() {
        if self.constraintsValid {
            return
        }

        for vertex in self.selectedVertices {
            var v = Vector2f()
            self.deformedMesh.getVertex(vertexIndex: vertex, vertex: &v)
            self.deformer.setDeformedHandle(handle: vertex, pos: Vector2f(v.x, v.y))
        }

        self.deformer.forceValidation()
        self.constraintsValid = true
    }

    private func findHitVertex(point: CGPoint, size: CGSize) -> Int? {
        let scale = 0.5 * min(size.width, size.height) / 2.0
        let translate = Vector2f(Float(size.width / 2), Float(size.height / 2))

        for i in 0 ..< self.deformedMesh.getVerticesCount() {
            var v = Vector2f()
            self.deformedMesh.getVertex(vertexIndex: i, vertex: &v)
            let viewPos = Vector2f(v.x, v.y) * Float(scale) + translate
            let dist = distance(Vector2f(Float(point.x), Float(point.y)), viewPos)
            if dist < 5 {
                return i
            }
        }
        return nil
    }

    public func handleDrag(point: CGPoint, size: CGSize) {
        if let selectedVertex = self.selectedVertex {
            let scale = 0.5 * min(size.width, size.height) / 2.0
            let translate = Vector2f(Float(size.width / 2), Float(size.height / 2))
            let worldPos = (Vector2f(Float(point.x), Float(point.y)) - translate) / Float(scale)
            self.deformedMesh.setVertex(vertexIndex: selectedVertex, vertex: Vector2f(worldPos.x, worldPos.y))
            self.invalidateConstraints()
        }
    }

    public func selectVertex(point: CGPoint, size: CGSize) {
        if let hit = findHitVertex(point: point, size: size) {
            self.selectedVertex = hit
        }
    }

    public func toggleSelection(point: CGPoint, size: CGSize) {
        if let hit = findHitVertex(point: point, size: size) {
            if self.selectedVertices.contains(hit) {
                self.selectedVertices.remove(hit)
                self.deformer.removeHandle(handle: hit)
                var v = Vector2f()
                self.initialMesh.getVertex(vertexIndex: hit, vertex: &v)
                self.deformedMesh.setVertex(vertexIndex: Int(hit), vertex: v)
            } else {
                self.selectedVertices.insert(hit)
            }
            self.invalidateConstraints()
        }
    }
    
    public func releaseSelection() {
        self.selectedVertex = nil
    }
}
