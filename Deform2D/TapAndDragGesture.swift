
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
                if !isDragging {
                    isDragging = true
                    startTime = Date()
                    onEvent(.drag(location: value.location, start: value.startLocation))
                } else {
                    onEvent(.drag(location: value.location, start: value.startLocation))
                }
            }
            .onEnded { value in
                if let startTime = startTime, Date().timeIntervalSince(startTime) < 0.2 { // Tap if duration is short
                    onEvent(.tap(location: value.startLocation))
                }
                onEvent(.release)
                isDragging = false
                self.startTime = nil
            }
    }
}
