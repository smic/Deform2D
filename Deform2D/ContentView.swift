//
//  ContentView.swift
//  Deform2D
//
//  Created by Stephan Michels on 05.07.25.
//

import SwiftUI

struct ContentView: View {
    @State private var viewModel = DeformationViewModel()

    var body: some View {
        GeometryReader { geometry in
            Canvas { context, size in
                viewModel.updateDeformedMesh()

                let scale = 0.5 * min(size.width, size.height) / 2.0
                let translate = SIMD2<Float>(Float(size.width / 2), Float(size.height / 2))

                for i in 0..<viewModel.deformedMesh.getNumTriangles() {
                    var triVerts = [SIMD3<Float>](repeating: .zero, count: 3)
                    var normals: [SIMD3<Float>]? = nil
                    viewModel.deformedMesh.getTriangle(triangleIndex: i, vTriangle: &triVerts, pNormals: &normals)

                    var path = Path()
                    let v1 = SIMD2<Float>(triVerts[0].x, triVerts[0].y) * Float(scale) + translate
                    let v2 = SIMD2<Float>(triVerts[1].x, triVerts[1].y) * Float(scale) + translate
                    let v3 = SIMD2<Float>(triVerts[2].x, triVerts[2].y) * Float(scale) + translate
                    path.move(to: CGPoint(x: CGFloat(v1.x), y: CGFloat(v1.y)))
                    path.addLine(to: CGPoint(x: CGFloat(v2.x), y: CGFloat(v2.y)))
                    path.addLine(to: CGPoint(x: CGFloat(v3.x), y: CGFloat(v3.y)))
                    path.closeSubpath()
                    context.stroke(path, with: .color(.black), lineWidth: 2)
                }

                for selected in viewModel.selectedVertices {
                    var v = SIMD3<Float>()
                    var n: SIMD3<Float>?
                    viewModel.deformedMesh.getVertex(vertexIndex: selected, vertex: &v, normal: &n)
                    let viewPos = SIMD2<Float>(v.x, v.y) * Float(scale) + translate
                    let rect = CGRect(x: CGFloat(viewPos.x - 5), y: CGFloat(viewPos.y - 5), width: 10, height: 10)
                    context.fill(Path(rect), with: .color(.red))
                }
            }
            .gesture(
                DragGesture(minimumDistance: 0)
                    .onChanged { value in
                        if viewModel.selectedVertices.isEmpty {
                            viewModel.selectVertex(point: value.startLocation, size: geometry.size)
                        }
                        viewModel.handleDrag(point: value.location, size: geometry.size)
                    }
                    .onEnded { _ in
                        viewModel.releaseSelection()
                    }
            )
            .gesture(
                TapGesture()
                    .onEnded { 
                        // This is a bit of a hack to get both tap and drag gestures to work
                        // a better solution would be to use a custom gesture recognizer
                        if let hit = viewModel.findHitVertex(point: CGPoint(x: 0,y: 0), size: geometry.size) {
                            viewModel.toggleSelection(point: CGPoint(x: 0,y: 0), size: geometry.size)
                        }
                    }
            )
        }
    }
}

#Preview {
    ContentView()
}
