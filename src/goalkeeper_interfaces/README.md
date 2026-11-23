# Goalkeeper Interfaces

Status: proposed module; custom messages and ROS 2 package files are not implemented.

Implement BallState, InterceptTarget, KeeperCommand, and KeeperStatus using the fields in the [interface specification](../../docs/interfaces.md). Register their message dependencies with rosidl and add package build metadata.

Use meters, radians, and ROS timestamps in messages. Preserve arrival time, expiry, sequence identifiers, and the distinction between estimated and measured states.
