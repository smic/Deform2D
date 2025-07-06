//
//  ContentView.swift
//  Deform2D
//
//  Created by Stephan Michels on 05.07.25.
//

import SwiftUI

struct ContentView: View {
    @State private var viewModel = DeformationViewModel()
    @State private var rigid: Bool = true
    @State private var dragStarted: Bool = false
    @State private var updateCounter: Int = 0

    var body: some View {
        // hack to update view
        let _ = self.updateCounter
        
        VStack {
            Toggle("Rigid", isOn: self.$rigid)
            
            GeometryReader { geometry in
                Canvas { context, size in
                    self.viewModel.updateDeformedMesh(rigid: self.rigid)
                    
                    let scale = 0.5 * min(size.width, size.height) / 2.0
                    let translate = Vector2f(Float(size.width / 2), Float(size.height / 2))
                    
                    for i in 0 ..< self.viewModel.deformedMesh.getTrianglesCount() {
                        var triangleVertices = [Vector2f](repeating: .zero, count: 3)
                        self.viewModel.deformedMesh.getTriangle(triangleIndex: i, vertices: &triangleVertices)
                        
                        var path = Path()
                        let v1 = Vector2f(triangleVertices[0].x, triangleVertices[0].y) * Float(scale) + translate
                        let v2 = Vector2f(triangleVertices[1].x, triangleVertices[1].y) * Float(scale) + translate
                        let v3 = Vector2f(triangleVertices[2].x, triangleVertices[2].y) * Float(scale) + translate
                        path.move(to: CGPoint(x: CGFloat(v1.x), y: CGFloat(v1.y)))
                        path.addLine(to: CGPoint(x: CGFloat(v2.x), y: CGFloat(v2.y)))
                        path.addLine(to: CGPoint(x: CGFloat(v3.x), y: CGFloat(v3.y)))
                        path.closeSubpath()
                        context.fill(path, with: .color(Color("FillColor")))
                        context.stroke(path, with: .color(Color("StrokeColor")), style: .init(lineWidth: 1, lineCap: .round, lineJoin: .round))
                    }
                    
                    for selected in self.viewModel.selectedVertices {
                        var v = Vector2f()
                        self.viewModel.deformedMesh.getVertex(vertexIndex: selected, vertex: &v)
                        let point = Vector2f(v.x, v.y) * Float(scale) + translate
                        let handleRadius: CGFloat = 3.5
                        let handleRect = CGRect(x: CGFloat(point.x) - handleRadius, y: CGFloat(point.y) - handleRadius, width: handleRadius * 2, height: handleRadius * 2)
                        let handlePath = Path(ellipseIn: handleRect)
                        
                        if selected == self.viewModel.selectedVertex {
                            context.fill(handlePath, with: .color(.yellow))
                        } else {
                            context.fill(handlePath, with: .color(.red))
                        }
                        context.stroke(handlePath, with: .color(.black), lineWidth: 1)
                    }
                }
                .gesture(
                    DragGesture(minimumDistance: 1)
                        .onChanged { value in
                            if !self.dragStarted {
                                self.viewModel.selectVertex(point: value.startLocation, size: geometry.size)
                                self.dragStarted = true
                            }
                            self.viewModel.handleDrag(point: value.location, size: geometry.size)
                            self.updateCounter += 1
                        }
                        .onEnded { value in
                            self.viewModel.releaseSelection()
                            self.dragStarted = false
                            self.updateCounter += 1
                        }
                        .simultaneously(with:
                            SpatialTapGesture()
                                .onEnded { event in
                                    self.viewModel.toggleSelection(point: event.location, size: geometry.size)
                                    self.updateCounter += 1
                                }
                        )
                )
            }
        }
    }
}

#Preview {
    ContentView()
}
