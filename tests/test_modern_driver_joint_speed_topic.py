#!/usr/bin/python
from __future__ import division
from IPython import embed
import numpy as np, rospy, copy, math, std_msgs, argparse, os, yaml
import matplotlib
import matplotlib.pyplot as plt
from catkin.find_in_workspaces import find_in_workspaces
from sensor_msgs.msg import JointState
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController

# $ roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=IP

# https://github.com/ros-industrial/ur_modern_driver#improvements
# /joint_speed : Takes messages of type trajectory_msgs/JointTrajectory. Parses the first JointTracetoryPoint and sends the specified joint speeds and accelerations to the robot. This interface is intended for doing visual servoing and other kind of control that requires speed control rather than position control of the robot. Remember to set values for all 6 joints. Ignores the field joint_names, so set the values in the correct order.

home_pose = [
    -1.1754127787170481e-05, -1.570775145281954, 2.0338028612711512e-05,
    -1.570784429061514, -1.0939200221216083e-05, 1.4323545606882795e-05
]


class Robot(object):
    def __init__(self):
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        self.num_dofs = len(self.joint_names)
        self.cur_jtpos = [0.0] * self.num_dofs
        self.cur_jtvel = [0.0] * self.num_dofs
        self.collecting_data_starting_time = None
        self.times = []
        self.jtposs = []
        self.jtvels = []
        joint_states_subscriber = rospy.Subscriber('/joint_states', JointState,
                                                   self.cur_jtstate_callback)

    def run(self, save=False):
        topic = "/ur_driver/joint_speed"
        pub = rospy.Publisher(topic, JointTrajectory, queue_size=10)
        rospy.sleep(1.0)

        traj = JointTrajectory()
        point = JointTrajectoryPoint()
        #  point.positions = copy.deepcopy(self.cur_jtpos)
        #  point.positions[0] += 0.1
        point.velocities = [0.11, 0.0, 0.0, 0.0, 0.0, 0.0]
        #  point.time_from_start = rospy.Time(1.0)
        traj.points.append(point)

        self.collecting_data_starting_time = rospy.Time.now()
        for i in range(100):
            pub.publish(traj)
            rospy.sleep(0.01)
        self.collecting_data_starting_time = None

        rospy.sleep(0.5)

        if not save:
            return

        # https://pythonspot.com/matplotlib-scatterplot/
        data = ((self.times, [x[0] for x in self.jtposs]),
                (self.times, [x[0] for x in self.jtvels]))
        colors = ("red", "green")
        groups = ("pos", "vel")
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        for i in range(len(data)):
            x, y = data[i]
            color = colors[i]
            group = groups[i]
            ax.plot(x, y, alpha=0.5, c=color, linewidth=4, label=group)
        plt.legend(loc=2)
        plt_name = "pos_vel"
        plt.title(plt_name)

        # plt.show()
        config_path = find_in_workspaces(
            search_dirs=['share'],
            project="ur_modern_driver",
            path="tests",
            first_match_only=True)
        path = os.path.join(config_path[0],
                            "test_modern_driver_joint_speed_topic_plots",
                            plt_name + ".pdf")
        print path
        plt.savefig(path)

    def cur_jtstate_callback(self, data):
        self.cur_jtpos = [0.0] * self.num_dofs
        self.cur_jtvel = [0.0] * self.num_dofs
        for i, jn in enumerate(self.joint_names):
            index = data.name.index(jn)
            self.cur_jtpos[i] = data.position[index]
            self.cur_jtvel[i] = data.velocity[index]
        if self.collecting_data_starting_time is not None:
            self.times.append((rospy.Time.now() -
                               self.collecting_data_starting_time).to_sec())
            self.jtposs.append(self.cur_jtpos)
            self.jtvels.append(self.cur_jtvel)


if __name__ == "__main__":
    rospy.init_node('test_vel_controller', anonymous=True)
    while not rospy.Time.now():
        pass

    r = Robot()
    r.run(save=False)
