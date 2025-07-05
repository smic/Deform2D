//
//  ContentView.swift
//  Deform2D
//
//  Created by Stephan Michels on 05.07.25.
//

import SwiftUI

struct ContentView: View {
    @State private var viewModel = DeformationViewModel()
    @State private var dragStarted: Bool = false
    @State private var updateCounter: Int = 0

    var body: some View {
        let _ = print("Update View:", self.updateCounter)
        let _ = Self._printChanges()
        GeometryReader { geometry in
            Canvas { context, size in
                self.viewModel.updateDeformedMesh()

                let scale = 0.5 * min(size.width, size.height) / 2.0
                let translate = SIMD2<Float>(Float(size.width / 2), Float(size.height / 2))

                for i in 0 ..< self.viewModel.deformedMesh.getNumTriangles() {
                    var triVertices = [SIMD3<Float>](repeating: .zero, count: 3)
                    var normals: [SIMD3<Float>]? = nil
                    self.viewModel.deformedMesh.getTriangle(triangleIndex: i, vTriangle: &triVertices, pNormals: &normals)

                    var path = Path()
                    let v1 = SIMD2<Float>(triVertices[0].x, triVertices[0].y) * Float(scale) + translate
                    let v2 = SIMD2<Float>(triVertices[1].x, triVertices[1].y) * Float(scale) + translate
                    let v3 = SIMD2<Float>(triVertices[2].x, triVertices[2].y) * Float(scale) + translate
                    path.move(to: CGPoint(x: CGFloat(v1.x), y: CGFloat(v1.y)))
                    path.addLine(to: CGPoint(x: CGFloat(v2.x), y: CGFloat(v2.y)))
                    path.addLine(to: CGPoint(x: CGFloat(v3.x), y: CGFloat(v3.y)))
                    path.closeSubpath()
                    context.fill(path, with: .color(Color("FillColor")))
                    context.stroke(path, with: .color(Color("StrokeColor")), style: .init(lineWidth: 1, lineCap: .round, lineJoin: .round))
                }

                print("self.viewModel.selectedVertices:", self.viewModel.selectedVertices)
                for selected in self.viewModel.selectedVertices {
                    var v = SIMD3<Float>()
                    var n: SIMD3<Float>?
                    self.viewModel.deformedMesh.getVertex(vertexIndex: selected, vertex: &v, normal: &n)
                    let point = SIMD2<Float>(v.x, v.y) * Float(scale) + translate
                    let handleRadius: CGFloat = 3.5
                    let handleRect = CGRect(x: CGFloat(point.x) - handleRadius, y: CGFloat(point.y) - handleRadius, width: handleRadius * 2, height: handleRadius * 2)
                    let handlePath = Path(ellipseIn: handleRect)
                    
//                    if index == selectedPointIndex {
//                        context.fill(handlePath, with: .color(.yellow))
//                    } else {
                        context.fill(handlePath, with: .color(.red))
//                    }
                    context.stroke(handlePath, with: .color(.black), lineWidth: 1)
                    
                    
//                    context.fill(Path(rect), with: .color(.red))
                }
            }
            .gesture(
                DragGesture(minimumDistance: 1)
                    .onChanged { value in
                        if !self.dragStarted {
                            viewModel.selectVertex(point: value.startLocation, size: geometry.size)
                            print("Drag started")
                            self.dragStarted = true
                        }
//                        if /*viewModel.selectedVertices.isEmpty*/viewModel.selectedVertex == nil {
//                            viewModel.selectVertex(point: value.location, size: geometry.size)
//                        }
                        viewModel.handleDrag(point: value.location, size: geometry.size)
                        self.updateCounter += 1
                    }
                    .onEnded { value in
                        viewModel.releaseSelection()
                        self.dragStarted = false
                        self.updateCounter += 1
                    }
                    .simultaneously(with:
                                        SpatialTapGesture()
                        .onEnded { event in
                            viewModel.toggleSelection(point: event.location, size: geometry.size)
                            self.updateCounter += 1
                        }
                                   )
                
                
//                TapAndDragGesture {
//                    event in
//                    switch event {
//                    case .tap(let location):
//                        viewModel.toggleSelection(point: location, size: geometry.size)
//                    case .drag(let location, let start):
//                        if viewModel.selectedVertices.isEmpty {
//                            viewModel.selectVertex(point: start, size: geometry.size)
//                        }
//                        viewModel.handleDrag(point: location, size: geometry.size)
//                    case .release:
//                        viewModel.releaseSelection()
//                    }
//                }
            )
        }
    }
}

#Preview {
    ContentView()
}
