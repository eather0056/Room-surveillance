import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MultiRobotNavigation(Node):
    def __init__(self):
        super().__init__('multi_robot_navigation')
        self.goal_pubs = {
            'tb3_6': self.create_publisher(PoseStamped, '/tb3_6/goal_pose', 10),
            'tb3_8': self.create_publisher(PoseStamped, '/tb3_8/goal_pose', 10),
            'tb3_9': self.create_publisher(PoseStamped, '/tb3_9/goal_pose', 10),
            'tb3_10': self.create_publisher(PoseStamped, '/tb3_10/goal_pose', 10),
        }
        self.timer = self.create_timer(2.0, self.send_goals)

    def send_goals(self):
        goals = {
            'tb3_6': {'x': 0.0, 'y': 3.5, 'z': 0.0},
            'tb3_8': {'x': 2.8, 'y': 3.0, 'z': 0.0},
            'tb3_9': {'x': 0.0, 'y': 3.5, 'z': 0.0},
            'tb3_10': {'x': 3.0, 'y': 0.0, 'z': 0.0},
        }
        for robot, goal in goals.items():
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = goal['x']
            pose.pose.position.y = goal['y']
            pose.pose.position.z = goal['z']
            pose.pose.orientation.w = 1.0
            self.goal_pubs[robot].publish(pose)
            self.get_logger().info(f'Sent goal to {robot}: {goal}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotNavigation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
