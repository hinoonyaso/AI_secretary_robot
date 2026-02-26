#!/usr/bin/env python3

import argparse
import sys


def build_parser():
    parser = argparse.ArgumentParser(description="Move the JetRover arm to a Cartesian target with MoveIt.")
    parser.add_argument("--group", default="arm", help="MoveIt planning group name")
    parser.add_argument("--frame", default="link1", help="Reference frame")
    parser.add_argument("--x", type=float, required=True, help="Target x (m)")
    parser.add_argument("--y", type=float, required=True, help="Target y (m)")
    parser.add_argument("--z", type=float, required=True, help="Target z (m)")
    parser.add_argument("--roll", type=float, default=0.0, help="Roll (rad)")
    parser.add_argument("--pitch", type=float, default=0.0, help="Pitch (rad)")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw (rad)")
    parser.add_argument("--planning-time", type=float, default=3.0, help="Planning timeout (sec)")
    parser.add_argument("--num-attempts", type=int, default=10, help="Planning attempts")
    parser.add_argument("--velocity-scale", type=float, default=0.3, help="Velocity scaling [0..1]")
    parser.add_argument("--accel-scale", type=float, default=0.3, help="Acceleration scaling [0..1]")
    return parser


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    args = build_parser().parse_args(argv)

    try:
        import moveit_commander
        from geometry_msgs.msg import Pose
        from tf_transformations import quaternion_from_euler
    except Exception as exc:
        print("[arm_moveit_commander] missing MoveIt python dependencies:", exc)
        print("Install moveit2 and python bindings, then source your ROS + workspace setup files.")
        return 1

    moveit_commander.roscpp_initialize(sys.argv)

    group = moveit_commander.MoveGroupCommander(args.group)
    group.set_pose_reference_frame(args.frame)
    group.set_planning_time(args.planning_time)
    group.set_num_planning_attempts(args.num_attempts)
    group.set_max_velocity_scaling_factor(max(0.01, min(1.0, args.velocity_scale)))
    group.set_max_acceleration_scaling_factor(max(0.01, min(1.0, args.accel_scale)))

    pose_goal = Pose()
    pose_goal.position.x = args.x
    pose_goal.position.y = args.y
    pose_goal.position.z = args.z

    qx, qy, qz, qw = quaternion_from_euler(args.roll, args.pitch, args.yaw)
    pose_goal.orientation.x = qx
    pose_goal.orientation.y = qy
    pose_goal.orientation.z = qz
    pose_goal.orientation.w = qw

    group.set_pose_target(pose_goal)

    plan = group.plan()

    # API compatibility across distributions.
    if isinstance(plan, tuple):
        success = bool(plan[0])
        trajectory = plan[1] if len(plan) > 1 else None
    else:
        success = bool(plan)
        trajectory = plan

    if not success or trajectory is None:
        print("[arm_moveit_commander] planning failed")
        group.clear_pose_targets()
        return 2

    executed = group.execute(trajectory, wait=True)
    group.stop()
    group.clear_pose_targets()

    if not executed:
        print("[arm_moveit_commander] trajectory execution failed")
        return 3

    print("[arm_moveit_commander] target reached")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
