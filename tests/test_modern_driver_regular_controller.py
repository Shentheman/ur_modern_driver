#!/usr/bin/python
from IPython import embed
import numpy as np, rospy, copy, math, std_msgs, argparse, os, yaml
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import matplotlib.pyplot as plt
from catkin.find_in_workspaces import find_in_workspaces

# $ roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=IP

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

        self.control_client = actionlib.SimpleActionClient(
            '/follow_joint_trajectory', FollowJointTrajectoryAction)
        assert (self.control_client.wait_for_server(rospy.Duration(10.0)))

    def test_control(self, time_gap_send_cmd_again=-1, save=False):
        freq = 500
        rate = rospy.Rate(freq)

        goal_pose = copy.deepcopy(self.cur_jtpos)
        goal_pose[0] -= 0.2
        pos2 = list(goal_pose)
        vel2 = [0.0] * self.num_dofs

        completion_time = 4.0
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.cur_jtpos
        point.velocities = self.cur_jtvel
        point.time_from_start = rospy.Time(0.0)
        goal.trajectory.points.append(point)

        point = JointTrajectoryPoint()
        point.positions = pos2
        point.velocities = vel2
        point.time_from_start = rospy.Time(completion_time)
        goal.trajectory.points.append(point)

        self.collecting_data_starting_time = rospy.Time.now()
        self.control_client.send_goal(goal)

        if time_gap_send_cmd_again > 1e-5:
            while True:
                rospy.sleep(time_gap_send_cmd_again)
                time_elapsed = (rospy.Time.now() -
                                self.collecting_data_starting_time).to_sec()
                if completion_time <= time_elapsed:
                    break

                goal = FollowJointTrajectoryGoal()
                goal.trajectory.joint_names = self.joint_names
                point = JointTrajectoryPoint()
                point.positions = self.cur_jtpos
                point.velocities = self.cur_jtvel
                point.time_from_start = rospy.Time(0.0)
                goal.trajectory.points.append(point)

                point = JointTrajectoryPoint()
                point.positions = pos2
                point.velocities = vel2
                point.time_from_start = rospy.Time(
                    completion_time - time_elapsed)
                goal.trajectory.points.append(point)
                self.control_client.send_goal(goal)
                print "Send"
        rospy.sleep(4.0)
        self.collecting_data_starting_time = None

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

        plt_name = "pos_vel" + str(time_gap_send_cmd_again)
        plt.title(plt_name)

        # plt.show()
        config_path = find_in_workspaces(
            search_dirs=['share'],
            project="ur_modern_driver",
            path="tests",
            first_match_only=True)
        path = os.path.join(config_path[0],
                            "test_modern_driver_regular_controller_plots",
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
    #  r.test_control(time_gap_send_cmd_again=-1, save=True)
    #  r.test_control(time_gap_send_cmd_again=0.6, save=True)
