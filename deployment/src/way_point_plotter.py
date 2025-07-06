#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import matplotlib.pyplot as plt
import matplotlib.animation as animation

class WaypointPlotter(Node):
    def __init__(self):
        super().__init__('waypoint_plotter')

        self.xs = []
        self.ys = []

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/waypoint',
            self.plotter_callback,
            10
        )

        # RViz publisher for waypoint markers
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # Setup matplotlib figure
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'bo-', label="Trajectory")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Live Waypoint Plot")
        self.ax.legend()
        self.ax.grid(True)

        # Set up animation
        self.ani = animation.FuncAnimation(
            self.fig,
            self.update_plot,
            interval=500
        )

    def plotter_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            x, y = msg.data[0], msg.data[1]
            self.get_logger().info(f"Received waypoint: ({x:.2f}, {y:.2f})")
            self.xs.append(x)
            self.ys.append(y)
            self.publish_marker_array()

    def publish_marker_array(self):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(zip(self.xs, self.ys)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def update_plot(self, frame):
        self.line.set_data(self.xs, self.ys)
        if self.xs and self.ys:
            self.ax.set_xlim(min(self.xs) - 0.1, max(self.xs) + 0.1)
            self.ax.set_ylim(min(self.ys) - 0.1, max(self.ys) + 0.1)
        return self.line,

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPlotter()

    # Run the ROS 2 node in a non-blocking way
    def spin():
        rclpy.spin_once(node, timeout_sec=0.01)

    timer = node.fig.canvas.new_timer(interval=10)
    timer.add_callback(spin)
    timer.start()

    # Start matplotlib event loop
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
