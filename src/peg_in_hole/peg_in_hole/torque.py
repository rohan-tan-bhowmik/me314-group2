#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import numpy as np
import math

class FTMonitor(Node):
    def __init__(self):
        super().__init__('ft_monitor')

        # Force & torque components
        self.FT_force_x = 0.0
        self.FT_force_y = 0.0
        self.FT_force_z = 0.0
        self.FT_torque_x = 0.0
        self.FT_torque_y = 0.0
        self.FT_torque_z = 0.0

        # Subscribe to external filtered FT sensor data
        self.create_subscription(
            WrenchStamped,
            '/xarm/uf_ftsensor_ext_states',
            self.ft_callback,
            10
        )

        # Log every 1 second
        self.create_timer(1.0, self.log_force_torque)

        self.get_logger().info("FT Monitor initialized. Subscribed to /xarm/uf_ftsensor_ext_states.")

    def ft_callback(self, msg: WrenchStamped):
        self.FT_force_x = msg.wrench.force.x
        self.FT_force_y = msg.wrench.force.y
        self.FT_force_z = msg.wrench.force.z
        self.FT_torque_x = msg.wrench.torque.x
        self.FT_torque_y = msg.wrench.torque.y
        self.FT_torque_z = msg.wrench.torque.z

    # --- Getters ---

    def get_all_ft(self):
        return {
            "force": {
                "x": self.FT_force_x,
                "y": self.FT_force_y,
                "z": self.FT_force_z
            },
            "torque": {
                "x": self.FT_torque_x,
                "y": self.FT_torque_y,
                "z": self.FT_torque_z
            }
        }

    def get_force_vector(self):
        return np.array([self.FT_force_x, self.FT_force_y, self.FT_force_z])

    def get_torque_vector(self):
        return np.array([self.FT_torque_x, self.FT_torque_y, self.FT_torque_z])

    def get_all_ft_flat(self):
        return (
            self.FT_force_x, self.FT_force_y, self.FT_force_z,
            self.FT_torque_x, self.FT_torque_y, self.FT_torque_z
        )

    # Optional periodic logger
    def log_force_torque(self):
        f_mag = np.linalg.norm(self.get_force_vector())
        t_mag = np.linalg.norm(self.get_torque_vector())
        self.get_logger().info(f"Force: {self.get_force_vector()}, Magnitude: {f_mag:.2f} N")
        self.get_logger().info(f"Torque: {self.get_torque_vector()}, Magnitude: {t_mag:.2f} Nm")


def main(args=None):
    rclpy.init(args=args)
    node = FTMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down FT Monitor.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
