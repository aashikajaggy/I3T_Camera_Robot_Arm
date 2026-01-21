import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyrealsense2 as rs
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS




class MarkerFinderSubscriber(Node):
    def __init__(self):
        super().__init__('marker_finder_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray, 'marker_location', self.listener_callback, 10)
        self.received_message = False
    def listener_callback(self, msg):
        if not self.received_message:
            self.get_logger().info(f'Coordinates received from intel realsense: {msg.data}')
            self.received_message = True
            transform_matrix = np.array([[0.0, 0.0, 1.0, 50.0], [1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 20.0], [0.0, 0.0, 0.0, 1.0]])
            coordinate_vector = np.array([msg.data[0], msg.data[1], msg.data[2], 1.0])
            transformed_matrix = np.matmul(transform_matrix, coordinate_vector)
            
            
            bot = InterbotixManipulatorXS(
                robot_model='px100',
                group_name='arm',
                gripper_name='gripper',
            )
            
            robot_startup()

            bot.gripper.grasp(2.0)
            bot.gripper.release(2.0)
            bot.gripper.set_pressure(1.0)
            bot.gripper.grasp(2.0)
            bot.gripper.release(2.0)

            robot_shutdown()

            


            print(f'result:{transformed_matrix}')
            return transformed_matrix
    
        
        

def main(args=None):
    rclpy.init(args=args)
    subscriber = MarkerFinderSubscriber()
    rclpy.spin(subscriber) #keeps the node active and listening for other messages 

    print(subscriber.listener_callback)
    

      

    
    '''
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper',
    )

    robot_startup()

    bot.arm.go_to_home_pose()
    bot.arm.set_ee_pose_components(x=0.2, y=0.1, z=0.2, roll=1.0, pitch=1.5)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

    robot_shutdown()
    '''


    subscriber.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

