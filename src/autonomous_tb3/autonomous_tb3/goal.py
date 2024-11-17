import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class MultiRobotNavigator(Node):
    def __init__(self):
        super().__init__('multi_robot_navigator')
        
        # Define goals for each robot
        self.goals = {
            'tb3_6': {'x': 0.0, 'y': 3.5, 'z': 0.0},
            'tb3_8': {'x': 2.8, 'y': 3.0, 'z': 0.0},
            'tb3_9': {'x': 0.0, 'y': 3.5, 'z': 0.0},
            'tb3_10': {'x': 3.0, 'y': 0.0, 'z': 0.0},
        }
        
        # Initialize publishers for each robot's goal topic
        self._publishers = {}
        for robot in self.goals.keys():
            self._publishers[robot] = self.create_publisher(
                PoseStamped,
                f'/{robot}/goal_pose',
                10
            )
        
        # Timer to send goals periodically
        self.timer = self.create_timer(2.0, self.send_goals)
        self.sent_goals = set()

    def send_goals(self):
        for robot, goal in self.goals.items():
            if robot not in self.sent_goals:
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = 'map'
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                goal_msg.pose.position.x = goal['x']
                goal_msg.pose.position.y = goal['y']
                goal_msg.pose.position.z = goal['z']
                goal_msg.pose.orientation.w = 1.0
                
                # Publish the goal
                self._publishers[robot].publish(goal_msg)
                self.get_logger().info(f"Sent goal to {robot}: {goal}")
                self.sent_goals.add(robot)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
