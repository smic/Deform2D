
import SwiftUI

struct TapAndDragGesture: Gesture {
    enum Event {
        case tap(location: CGPoint)
        case drag(location: CGPoint, start: CGPoint)
        case release
    }

    var onEvent: (Event) -> Void

    @State private var isDragging = false
    @State private var startTime: Date? = nil

    public var body: some Gesture {
        DragGesture(minimumDistance: 0)
            .onChanged { value in
                if !self.isDragging {
                    self.isDragging = true
                    self.startTime = Date()
                    self.onEvent(.drag(location: value.location, start: value.startLocation))
                } else {
                    self.onEvent(.drag(location: value.location, start: value.startLocation))
                }
            }
            .onEnded { value in
                if let startTime = startTime, Date().timeIntervalSince(startTime) < 0.2 { // Tap if duration is short
                    self.onEvent(.tap(location: value.startLocation))
                }
                self.onEvent(.release)
                self.isDragging = false
                self.startTime = nil
            }
    }
}
