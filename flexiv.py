import flexivrdk
import rospy
import numpy as np
import spdlog
import time
from realrobot.constants import (
    ROBOT_IP,                 
    ROBOT_POSITIONS,
    ROBOT_DOF,
    ROBOT_JOINT_STATE_TOPIC,
    ROBOT_COMMANDED_JOINT_STATE_TOPIC,
    ROBOT_EE_POSE_TOPIC,
)
from realrobot.utils.network import JointStatePublisher, FloatArrayPublisher
from scipy.spatial.transform import Rotation as R


class FlexivArm:
    """
    A wrapper class to control the Flexiv robotic arm.
    Provides methods to initialize the connection, control the arm, and publish its state.
    """

    def __init__(self):
        self.robot_sn = "Rizon4-062521"  # Serial number of the Flexiv robot
        self.logger = spdlog.ConsoleLogger("RobotController")
        self.initialize_connection()     # Establish connection to the robot
        self.vel = 1.0  # Default velocity
        self.dof = 7    # Degrees of freedom for the robot arm

        # Publishers for joint states and end-effector pose
        self.joint_state_publisher = JointStatePublisher(
            publisher_name=ROBOT_JOINT_STATE_TOPIC
        )
        self.command_joint_state_publisher = JointStatePublisher(
            publisher_name=ROBOT_COMMANDED_JOINT_STATE_TOPIC
        )
        self.ee_pose_publisher = FloatArrayPublisher(
            publisher_name=ROBOT_EE_POSE_TOPIC
        )

    def initialize_connection(self):
        """
        Initializes the connection to the Flexiv robot.
        Clears any faults and ensures the robot is operational.
        """
        mode = flexivrdk.Mode
        try:
            # Connect to the robot
            self.flexiv = flexivrdk.Robot(self.robot_sn)

            # Clear any faults if they exist
            if self.flexiv.fault():
                self.logger.warn("Fault occurred on the connected robot")

                if not self.flexiv.ClearFault():
                    return 1
                self.logger.info("Fault cleared successfully")

            # Enable the robot and set mode
            self.flexiv.Enable()
            self.flexiv.SwitchMode(mode.NRT_CARTESIAN_MOTION_FORCE)

            # Wait until the robot is operational
            while not self.flexiv.operational():
                time.sleep(1)

        except Exception as e:
            self.logger.error(f"Failed to connect to Flexiv robot: {e}")

    def home_robot(self):
        """
        Moves the robot to its home position.
        """
        # TODO: Implement the home position for the Flexiv robot

        raise NotImplementedError("Home position not implemented for Flexiv robot")

    def get_tcp_position(self, euler=False, degree=True):
        """
        Retrieves the current TCP (Tool Center Point) position of the robot.

        Args:
            euler (bool): If True, returns Euler angles instead of quaternion for rotation.
            degree (bool): If True, Euler angles are returned in degrees.

        Returns:
            list: TCP position (translation + rotation in either quaternion or Euler angles).
        """
        try:
            tcp_position = self.flexiv.states().tcp_pose
            print(f"Current TCP quaternion: {tcp_position}")

            if euler:
                rot = self.quat2eulerZYX(tcp_position[3:], degree)
                trans = tcp_position[:3]
                return trans + rot

            return tcp_position
        except Exception as e:
            self.logger.error(f"Failed to get TCP position: {e}")
            return None

    def move(self, target_arm_pose):
        """
        Sends a command to move the robot to a target pose.

        Args:
            target_arm_pose (list): The desired target pose [x, y, z, qx, qy, qz, qw].
        """
        try:
            self.flexiv.SendCartesianMotionForce(
                target_arm_pose, [0] * 6, 0.5, 1.0, 0.4
            )
        except Exception as e:
            self.logger.error(f"Failed to move the robot: {e}")

    def quat2eulerZYX(self, quat, degree=False):
        """
        Converts a quaternion to ZYX Euler angles.

        Args:
            quat (list): Quaternion [qw, qx, qy, qz].
            degree (bool): If True, returns angles in degrees.

        Returns:
            list: ZYX Euler angles.
        """
        eulerZYX = (
            R.from_quat([quat[1], quat[2], quat[3], quat[0]])
            .as_euler("xyz", degrees=degree)
            .tolist()
        )
        return eulerZYX

    def eulerZYX2quat(self, euler, degree=False):
        """
        Converts ZYX Euler angles to a quaternion.

        Args:
            euler (list): Euler angles [roll, pitch, yaw].
            degree (bool): If True, input angles are in degrees.

        Returns:
            list: Quaternion [qw, qx, qy, qz].
        """
        if degree:
            euler = np.radians(euler)

        tmp_quat = R.from_euler("xyz", euler).as_quat().tolist()
        quat = [tmp_quat[3], tmp_quat[0], tmp_quat[1], tmp_quat[2]]
        return quat

    def publish_state(self, input_cmd=None):
        """
        Publishes the robot's current TCP pose and optionally the commanded joint state.

        Args:
            input_cmd (list): Commanded joint positions (optional).
        """
        try:
            current_ee_pose = self.flexiv.states().tcp_pose
        except Exception as e:
            self.logger.error(f"Failed to get TCP position: {e}")
            return None

        # Publish TCP pose
        self.ee_pose_publisher.publish(current_ee_pose)

        # Publish commanded joint state, if provided
        if input_cmd is not None:
            self.command_joint_state_publisher.publish(input_cmd)


# Example usage
if __name__ == "__main__":
    rospy.init_node("flexiv_arm_controller")
    controller = FlexivArm()

    # Get the current TCP position in Euler angles
    tcp = controller.get_tcp_position(euler=True)
    print(tcp)
