import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

class RobocarStatePublisher(Node):

    def __init__(self):
    
        super().__init__('node_robocar_state_pub')      #node name 
        qos_profile = QoSProfile(depth=10)
        
        #joint subscriber
        self.joint_sub = self.create_subscription(Int32MultiArray,'whValue',self.wheel_callback,10)
        #type: JointState, topic: joint_statesのpublisherを作成
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        #robot state
        joint_state = JointState()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()        
        joint_state.name =['left_wheel_joint', 'right_wheel_joint'] 
        left_wheel_joint = 0.
        right_wheel_joint =0.
        joint_state.position = [left_wheel_joint, right_wheel_joint]    
        self.joint_pub.publish(joint_state)
     

    def wheel_callback(self, msg):
        joint_state = JointState()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()        
        joint_state.name =['left_wheel_joint', 'right_wheel_joint'] 
        left_wheel_joint = msg.data[0]*3.14/180.
        right_wheel_joint = msg.data[1]*3.14/180.
        joint_state.position = [left_wheel_joint, right_wheel_joint]    
        self.joint_pub.publish(joint_state)
              
       
def main(args=None):
    rclpy.init(args=args)
    robocar_state_pub = RobocarStatePublisher()
    rclpy.spin(robocar_state_pub)
    
    #robocar_state_pub.destroy_node()
    #rclpy.shutdown()

if __name__ == '__main__':
    main()
