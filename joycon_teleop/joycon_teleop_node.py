import time

import glm
import numpy as np
import pyjoycon
import rclpy
import rerun as rr
from geometry_msgs.msg import TwistStamped, PoseStamped
from glm import angleAxis
from pyjoycon import GyroTrackingJoyCon
from pyjoycon import get_device_ids
from rclpy.node import Node


class FullJoyCon(pyjoycon.GyroTrackingJoyCon, pyjoycon.ButtonEventJoyCon):
    pass


class JoyconTeleopNode(Node):

    def __init__(self):
        super().__init__('joycon_teleop')
        self.twist_pub = self.create_publisher(TwistStamped, 'twist', 1)
        self.direction_pub = self.create_publisher(PoseStamped, 'pointer', 1)

        self.period = 1 / 86  # pulled from gyro.py
        self.i = 0

        joycon_id = get_device_ids(debug=True)[0]
        self.joycon = GyroTrackingJoyCon(*joycon_id)

        # wait for data to come in
        time.sleep(0.5)

        self.accel_calib_offset = np.zeros(3)
        self.vel = np.zeros(3)
        self.reset()

        self.ready = False

        self.timer = self.create_timer(self.period, self.timer_callback)

        self.stick_right_vertical_min = 0.0
        self.stick_right_vertical_max = 0.0
        self.stick_right_horizontal_min = 0.0
        self.stick_right_horizontal_max = 0.0

        self.calibrate_right_stick()

    def reset(self):
        self.joycon.reset_orientation()
        self.joycon.calibrate()
        self.vel = np.zeros(3)
        self.accel_calib_offset = np.array(self.joycon.accel_in_g).mean(axis=0)

    def calibrate_right_stick(self):
        stick_calib_delay = 3
        print("Calibrating right stick...")
        print("Move the right stick to the top")
        time.sleep(stick_calib_delay)
        self.stick_right_vertical_max = self.joycon.get_stick_right_vertical()
        print("Move the right stick to the bottom")
        time.sleep(stick_calib_delay)
        self.stick_right_vertical_min = self.joycon.get_stick_right_vertical()
        print("Move the right stick to the left")
        time.sleep(stick_calib_delay)
        self.stick_right_horizontal_min = self.joycon.get_stick_right_horizontal()
        print("Move the right stick to the right")
        time.sleep(stick_calib_delay)
        self.stick_right_horizontal_max = self.joycon.get_stick_right_horizontal()
        print("Calibration complete")

    def timer_callback(self):
        try:
            # print("joycon pointer:  ", self.joycon.pointer)
            # print("joycon rotation: ", self.joycon.rotation)
            # print("joycon direction:", self.joycon.direction)

            # visualize pointer by creating an image and drawing at "pointer" location
            w, h = 200, 200
            img = np.zeros((w, h, 3), dtype=np.uint8)
            px = int(self.joycon.pointer[0] * w + w // 2)
            py = int(-self.joycon.pointer[1] * h + h // 2)
            s = 2
            img[py - s:py + s, px - s:px + s] = 255

            rr.log("pointer_image", rr.Image(img))

            # average the latest readings
            accel_in_g = np.array(self.joycon.accel_in_g).mean(axis=0)
            # remove calibration offset, which includes gravity
            accel_in_g -= self.accel_calib_offset
            # filter out small values before integrating
            accel_in_g[abs(accel_in_g) < 0.01] = 0
            # self.vel += accel_in_g * self.period
            gyro_in_rad = np.array(self.joycon.gyro_in_rad).mean(axis=0)

            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = "vr"
            # twist_msg.twist.linear.x = self.vel[0]
            # twist_msg.twist.linear.y = self.vel[1]
            # twist_msg.twist.linear.z = self.vel[2]
            twist_msg.twist.angular.x = gyro_in_rad[0]
            twist_msg.twist.angular.y = gyro_in_rad[1]
            twist_msg.twist.angular.z = gyro_in_rad[2]
            self.twist_pub.publish(twist_msg)

            orientation = glm.quat()
            for gx, gy, gz in self.joycon.gyro_in_rad:
                rotation = angleAxis(gx * (-self.period), self.joycon.direction_X) * \
                           angleAxis(gy * (-self.period), self.joycon.direction_Y) * \
                           angleAxis(gz * (-self.period), self.joycon.direction_Z)
                orientation *= rotation

            direction_pose = PoseStamped()
            direction_pose.header.stamp = self.get_clock().now().to_msg()
            direction_pose.header.frame_id = "vr"
            direction_pose.pose.orientation.x = self.joycon.direction_Q.x
            direction_pose.pose.orientation.y = self.joycon.direction_Q.y
            direction_pose.pose.orientation.z = self.joycon.direction_Q.z
            direction_pose.pose.orientation.w = self.joycon.direction_Q.w
            self.direction_pub.publish(direction_pose)

            right_stick_v = (self.joycon.get_stick_right_vertical() - self.stick_right_vertical_min) / (
                    self.stick_right_vertical_max - self.stick_right_vertical_min)
            right_stick_h = (self.joycon.get_stick_right_horizontal() - self.stick_right_horizontal_min) / (
                    self.stick_right_horizontal_max - self.stick_right_horizontal_min)

            # TODO: try simulating the pose of the end-effector motion
            #  where we use the right_stick_v to along in the forward (+X? +Z?) direction

            rr.log("stick/vertical", rr.TimeSeriesScalar(right_stick_v))
            rr.log("stick/horizontal", rr.TimeSeriesScalar(right_stick_h))
            rr.log("pointer/x", rr.TimeSeriesScalar(self.joycon.pointer[0]))
            rr.log("pointer/y", rr.TimeSeriesScalar(self.joycon.pointer[1]))
            rr.log("accel/x", rr.TimeSeriesScalar(accel_in_g[0]))
            rr.log("accel/y", rr.TimeSeriesScalar(accel_in_g[1]))
            rr.log("accel/z", rr.TimeSeriesScalar(accel_in_g[2]))
            rr.log("rotation/x", rr.TimeSeriesScalar(self.joycon.rotation[0]))
            rr.log("rotation/y", rr.TimeSeriesScalar(self.joycon.rotation[1]))
            rr.log("rotation/z", rr.TimeSeriesScalar(self.joycon.rotation[2]))
            rr.log("direction/x", rr.TimeSeriesScalar(self.joycon.direction[0]))
            rr.log("direction/y", rr.TimeSeriesScalar(self.joycon.direction[1]))
            rr.log("direction/z", rr.TimeSeriesScalar(self.joycon.direction[2]))

            # Log 3D arrows for the rotation and direction
            rr.log("rotation", rr.Arrows3D(vectors=list(self.joycon.rotation), origins=np.zeros(3)))
            rr.log("direction", rr.Arrows3D(vectors=list(self.joycon.direction), origins=np.zeros(3)))
            rr.log("origin", rr.Transform3D())
        except TypeError:
            pass


def main(args=None):
    rr.init('joycon_teleop')
    rr.connect()
    rclpy.init(args=args)

    node = JoyconTeleopNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
