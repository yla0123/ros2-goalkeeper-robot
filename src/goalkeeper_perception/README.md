# Goalkeeper Perception

Status: proposed module; executable nodes and ROS 2 package files are not implemented.

Implement a detector, calibrated camera-to-goal transform, tracker, and goal crossing predictor. Begin with a fixed camera and colored-ball detection for controlled trials.

Publish BallState in goal_frame with observation timestamps. Predict an InterceptTarget containing lateral position, height, arrival time, and validity. Use a low rolling-ball model first; add a gravity-aware model for airborne shots.

See the [interface specification](../../docs/interfaces.md) and [reuse decisions](../../docs/reuse-decisions.md).
