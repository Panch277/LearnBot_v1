import ROSLIB from 'roslib';
import { rosConnection } from './rosConnection';

// RViz's own "2D Pose Estimate" tool uses this same diagonal covariance by
// default (x/y variance 0.25, yaw variance ~0.0685) -- a reasonable amount
// of initial uncertainty for AMCL to relax into, not a value we're deriving
// from anything robot-specific.
const DEFAULT_INITIAL_POSE_COVARIANCE = [
  0.25, 0, 0, 0, 0, 0,
  0, 0.25, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0.06853892326654787,
];

// Created once at module load, not per publish: a fresh ROSLIB.Topic makes
// rosbridge advertise a brand-new ROS2 publisher, and DDS discovery needs a
// moment to match that against AMCL's existing subscriber. Publish
// immediately after creating it (as a naive per-click `new Topic()` here
// would) and that first message goes out before the match completes and is
// silently dropped -- the exact reason `ros2 topic pub` itself waits for
// "at least 1 matching subscription" before sending. Creating these at
// module scope instead gives discovery plenty of time before any click ever
// happens, the same pattern teleop.ts's /cmd_vel_joy topic already uses.
const initialPoseTopic = new ROSLIB.Topic({
  ros: rosConnection.ros,
  name: '/initialpose',
  messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
});

const goalPoseTopic = new ROSLIB.Topic({
  ros: rosConnection.ros,
  name: '/goal_pose',
  messageType: 'geometry_msgs/msg/PoseStamped',
});

// Both take a map-frame (x, y) plus a yaw (radians, counter-clockwise from
// +X) -- the drag direction after the initial click, mirroring RViz/
// Foxglove's "2D Pose Estimate" arrow. yaw defaults to 0 (identity
// orientation) for callers that only care about position: AMCL converges
// heading from subsequent motion either way, so a plain click without a
// drag still works, it just leaves the arrow pointing along +X.
function yawToQuaternion(yaw: number): { x: number; y: number; z: number; w: number } {
  return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) };
}

export function publishInitialPose(x: number, y: number, yaw = 0): void {
  initialPoseTopic.publish(
    new ROSLIB.Message({
      header: { frame_id: 'map' },
      pose: {
        pose: {
          position: { x, y, z: 0 },
          orientation: yawToQuaternion(yaw),
        },
        covariance: DEFAULT_INITIAL_POSE_COVARIANCE,
      },
    }),
  );
}

export function publishGoalPose(x: number, y: number, yaw = 0): void {
  goalPoseTopic.publish(
    new ROSLIB.Message({
      header: { frame_id: 'map' },
      pose: {
        position: { x, y, z: 0 },
        orientation: yawToQuaternion(yaw),
      },
    }),
  );
}
