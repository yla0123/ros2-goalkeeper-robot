# Integration Plan

1. Generate consistent goal and keeper assets and check the goal aperture, mesh, joint limits, and swept clearance.
2. Use the standard launch and check measured slide and tilt feedback during the [motion demo](robot-design.md#run-and-check).
3. **Run scripted trials:** capture measured joint tracking, arrival timing and scoring for centre, low and high-corner shots. Compare planned reachability with observed motion; record the cause only when supported by contact or trajectory data.
4. **Tune dynamics and repeat:** measure acceleration, command latency and overshoot; adjust Gazebo gains and limits using logs. A full wheel model with traction can replace the ideal slide when needed.
5. **Connect perception:** calibrate the camera-to-goal transform; track and predict the ball; publish coordinates and arrival timestamps using the [active interface](interfaces.md). Evaluate prediction accuracy separately from scripted shots.
