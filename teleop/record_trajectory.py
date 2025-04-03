import numpy as np
import argparse
import time
from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.utils import min_jerk, min_jerk_weight,convert_rigid_transform_to_array
from spacemouse import Spacemouse
from scipy.spatial.transform import Rotation as R
import rospy
import pickle as pkl


def create_formated_skill_dict(joints, end_effector_positions, gripprt_pos,time_since_skill_started):
    skill_dict = dict(skill_description='GuideMode', skill_state_dict=dict())
    skill_dict['skill_state_dict']['q'] = np.array(joints)
    skill_dict['skill_state_dict']['O_T_EE'] = np.array(end_effector_positions)
    skill_dict['skill_state_dict']['g_p'] = np.array(gripprt_pos)
    skill_dict['skill_state_dict']['time_since_skill_started'] = np.array(time_since_skill_started)

    # The key (0 here) usually represents the absolute time when the skill was started but
    formatted_dict = {0: skill_dict}
    return formatted_dict

def activate(fa, duration=10):
    fa.goto_pose(fa.get_pose(), duration=5, dynamic=True, buffer_time=duration,
        cartesian_impedances=FC.DEFAULT_TRANSLATIONAL_STIFFNESSES + FC.DEFAULT_ROTATIONAL_STIFFNESSES
    )

def publish_tgt_pose(fa, pose, pub):
    timestamp = rospy.Time.now().to_time()

    traj_gen_proto_msg = PosePositionSensorMessage(
        id=-1, timestamp=timestamp, 
        position=pose.translation, quaternion=pose.quaternion
    )
    fb_ctrlr_proto = CartesianImpedanceSensorMessage(
        id=-1, timestamp=timestamp,
        translational_stiffnesses=FC.DEFAULT_TRANSLATIONAL_STIFFNESSES,
        rotational_stiffnesses=FC.DEFAULT_ROTATIONAL_STIFFNESSES
    )
    ros_msg = make_sensor_group_msg(
        trajectory_generator_sensor_msg=sensor_proto2ros_msg(
            traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
        feedback_controller_sensor_msg=sensor_proto2ros_msg(
            fb_ctrlr_proto, SensorDataMessageType.CARTESIAN_IMPEDANCE)
        )
    
    pub.publish(ros_msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--time', '-t', type=float, default=35)
    parser.add_argument('--open_gripper', '-o', action='store_true')
    parser.add_argument('--file', '-f', default='franka_traj.pkl')
    args = parser.parse_args()

    fa = FrankaArm()
    if args.open_gripper:
        fa.open_gripper()
    fa.reset_joints()

    print('Ready...')


    dt = 0.02
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    rate = rospy.Rate(1 / dt)

    activate(fa, duration=50)
    print('Activated...')

    max_pos_speed=0.15
    max_rot_speed=0.25

    target_pose = fa.get_pose()
    gripper_open = True
    end_effector_position = []
    joints = []
    gripper_pos = []
    time_since_skill_started = []
    start_time = time.time()
    last_time = None
    with Spacemouse(deadzone=0.3) as sm:
        while last_time is None or (last_time - start_time) < args.time:
            # record trajectory
            pose_array = convert_rigid_transform_to_array(fa.get_pose())
            end_effector_position.append(pose_array)
            joints.append(fa.get_joints())
            
            # get gripper width
            width=fa.get_gripper_width()
            gripper_pos.append(width)
            
            time_since_skill_started.append(time.time() - start_time)
            
            # calculate target trajectory
            sm_state = sm.get_motion_state_transformed()
            # print(sm_state)
            dpos = sm_state[:3] * (max_pos_speed * dt)
            drot_xyz = sm_state[3:] * (max_rot_speed * dt)

            if not sm.is_button_pressed(0):
                # translation mode
                drot_xyz[:] = 0
            else:
                dpos[:] = 0

            if sm.is_button_pressed(1):
                if gripper_open:
                    fa.close_gripper()
                    gripper_open = False
                else:
                    fa.open_gripper()
                    gripper_open = True

            drot = R.from_euler('xyz', drot_xyz)
            target_pose.translation += dpos
            target_pose.rotation = (drot * R.from_matrix(target_pose.rotation)).as_matrix()

            publish_tgt_pose(fa, target_pose, pub)
            rate.sleep()
            last_time = time.time()
            
    skill_dict = create_formated_skill_dict(joints, end_effector_position,gripper_pos, time_since_skill_started)
    with open(args.file, 'wb') as pkl_f:
        pkl.dump(skill_dict, pkl_f)
        print("Did save skill dict: {}".format(args.file))


