#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from commander.srv import Position

import ikfastpy
import numpy as np
from scipy.spatial.transform import Rotation as R


class controller_command:
    def __init__(self):
        self.ur5_kin = ikfastpy.PyKinematics()
        self.n_joints = self.ur5_kin.getDOF()

        rospy.init_node('controller_command', anonymous=True)
        self.hz = 100
        self.rate = rospy.Rate(self.hz)

        self.pub = rospy.Publisher('scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.sub = rospy.Subscriber('scaled_pos_joint_traj_controller/state', JointTrajectoryControllerState,
                                    self.get_current_state_cb)
        self.sleep(0.3)
        self.current_state = None
        self.joint_names = None
        while self.current_state is None:
            pass
        print("controller_command init finished")

    def move(self, p, v_scale=0.1, duration_low_bound=1, start_duration=0.5):
        target_q = self.get_inverse_kin(p)
        if target_q is None:
            return False

        traj = JointTrajectory()
        traj.header = rospy.Header(frame_id="1", stamp=rospy.Time())
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
                            'wrist_2_joint',
                            'wrist_3_joint']
        dist = np.abs(np.subtract(target_q, self.current_state.positions)).max()
        duration = rospy.Duration(nsecs=int(max(dist / v_scale, duration_low_bound) * 1e9))
        # print(dist,duration.secs,duration.nsecs)
        end_point = JointTrajectoryPoint(
            positions=target_q,
            velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            time_from_start=duration
        )
        start_point = JointTrajectoryPoint(
            positions=(np.array(self.current_state.positions) + np.array(self.current_state.velocities) * (
                        start_duration + 0.01)).tolist(),
            velocities=self.current_state.velocities,
            time_from_start=rospy.Duration(0, int(start_duration * 1e9))
        )
        traj.points = [start_point, end_point]
        self.pub.publish(traj)
        return True

    def get_current_state_cb(self, state):
        self.current_state = state.actual
        self.joint_names = state.joint_names

    def sleep(self, sec):
        cnt = int(sec / (1.0 / self.hz))
        for _ in range(cnt):
            self.rate.sleep()

    def get_inverse_kin(self, p):
        r = R.from_rotvec(p[3:])
        rmat = r.as_matrix()
        trans = np.concatenate([rmat, np.reshape(p[:3], (3, 1))], axis=-1)
        joint_configs = self.ur5_kin.inverse(trans.reshape(-1).tolist())
        if len(joint_configs) == 0:
            print("inverse kinematics no solution")
            return None
        joint_configs = np.array(joint_configs).reshape(-1, self.n_joints)
        dist = (np.subtract(joint_configs, self.current_state.positions) ** 2).sum(axis=-1) ** 0.5
        return joint_configs[np.argmin(dist)].tolist()

    def get_forward_kin(self, q):
        trans = np.array(self.ur5_kin.forward(q)).reshape(3, 4)
        xyz = trans[:, 3].reshape(-1).tolist()
        r_xyz = R.from_matrix(trans[:, :3]).as_rotvec().tolist()
        return xyz + r_xyz


def command():
    commander = controller_command()

    def move_to_cb(req):
        return commander.move(req.joint_positions, req.v_scale, req.duration_low_bound, req.start_duration)

    service = rospy.Service('command_move_to', Position, move_to_cb, buff_size=10)
    rospy.spin()


if __name__ == '__main__':
    try:
        command()
    except rospy.ROSInterruptException:
        pass
