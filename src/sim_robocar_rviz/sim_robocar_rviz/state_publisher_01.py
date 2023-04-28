from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher_01')      #node name 

        qos_profile = QoSProfile(depth=10)
        #type: JointState, topic: joint_statesのpublisherを作成
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        left_wheel_joint = 0.            #joint
        right_wheel_joint = 0.
        tinc = degree
        angle = 0.
        hinc = 0.005           

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odome'
        odom_trans.child_frame_id = 'base_link'    #urdfで定義されたbase_link frame
        joint_state = JointState()            #JointState型の変数を作成

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
                joint_state.position = [left_wheel_joint, right_wheel_joint]

                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = left_wheel_joint*0.032
                odom_trans.transform.translation.y = 0.0
                odom_trans.transform.translation.z = 0.0
                odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # Create new robot state
                left_wheel_joint+= degree/4
                right_wheel_joint+= degree/4
                angle = 0

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()
