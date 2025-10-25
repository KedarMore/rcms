import rclpy
from rclpy.node import Node
from robot_msgs.msg import RobotState, PathComplete
import time
class TestNode(Node):
    def __init__(self,n):
        super().__init__("Test_Node"+str(n),allow_undeclared_parameters=True)
        self.message_publisher = self.create_publisher(
            RobotState, "robot/robot_state", 10)
        print("publisher initialised")
        self.publisher_timer = self.create_timer(
            5, self.publish_message)
    
    def publish_message(self):
        print("Publishing message")
        msg = RobotState()
        msg.hardware_id = "AArobot_0"
        msg.hardware_model= ''
        msg.robot_id= "robot_0"
        msg.robot_mode= 'autonomous'
        msg.robot_config_version= ''
        msg.robot_software_version= ''
        msg.robot_firmware_version= ''
        msg.uid= 4626
        msg.x= 7.056878566741943
        msg.y= 6.7
        msg.theta= 3.1122872829437256
        msg.v= 0.3250499665737152
        msg.w= 0.0890795961022377
        msg.control_status= 0
        msg.parcel_status= 'empty'
        msg.charge= 30.0
        msg.error_status= 'none'
        msg.path_uid= 0
        msg.path_error= False
        msg.robot_mode_status= 0
        msg.timestamp= 4567898
        msg.robot_task_id= ''
        msg.robot_task_status= 0
        msg.log_key= False
        msg.log_data= ''
        msg.robot_lift_height= 0.0
        self.message_publisher.publish(msg)

def runNode1(args=None):
    rclpy.init(args=args)
    testNode1= TestNode(1)
    rclpy.spin(testNode1)
    rclpy.shutdown()