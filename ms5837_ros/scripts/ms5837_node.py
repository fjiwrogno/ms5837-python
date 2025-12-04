#!/usr/bin/env python

import rospy
import ms5837
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Empty, EmptyResponse

class MS5837Node:
    def __init__(self):
        rospy.init_node('depth_sensor_node')

        # Parameters
        self.bus = 0
        self.fluid_density = 1000 # kg/m^3
        self.rate_hz = 100
        self.filter_factor = rospy.get_param('~filter_factor', 0.6) # 0.1-1.0, smaller = smoother
        
        # Outlier Rejection Parameters
        self.max_depth = 10.0
        self.min_depth = -10.0 # Allow negative for surface/air
        self.max_diff = 0.1 # Max change per sample (m)

        # Statew
        self.depth_offset = 0.0
        self.filtered_depth = None

        # Initialize sensor
        try:
            self.sensor = ms5837.MS5837_30BA(self.bus)
            if not self.sensor.init():
                rospy.logerr("Sensor could not be initialized")
                exit(1)
            
            self.sensor.setFluidDensity(self.fluid_density)
        except Exception as e:
            rospy.logerr("Failed to initialize sensor: %s" % str(e))
            exit(1)

        # Publishers
        # self.pressure_pub = rospy.advertise('~pressure', FluidPressure, queue_size=1)
        self.depth_pub = rospy.advertise('~depth', PointStamped, queue_size=1)
        
        # Services
        self.calibrate_srv = rospy.advertise_service('~calibrate', Empty, self.calibrate_callback)

        self.rate = rospy.Rate(self.rate_hz)

        # OSR Settings
        self.OSR_FAST = ms5837.OSR_1024
        self.OSR_BALANCE_100hz = ms5837.OSR_2048
        self.OSR_PRECISE = ms5837.OSR_8192

    def calibrate_callback(self, req):
        if self.filtered_depth is not None:
            self.depth_offset = self.filtered_depth
            rospy.loginfo("Depth sensor calibrated. New offset: %.4f m", self.depth_offset)
        else:
            rospy.logwarn("Cannot calibrate: no depth data received yet.")
        return EmptyResponse()

    def run(self):
        while not rospy.is_shutdown():
            # Use OSR_FAST (1024) for ~200Hz max rate, fitting well within 100Hz loop
            current_osr = self.OSR_BALANCE_100hz

            if self.sensor.read(oversampling=current_osr):
                # Raw depth from sensor
                raw_depth = self.sensor.depth()

                # --- Outlier Rejection ---
                # 1. Range Check
                if raw_depth < self.min_depth or raw_depth > self.max_depth:
                    rospy.logwarn_throttle(10, "Depth out of range: %.2f (Limits: %.1f, %.1f)" % (raw_depth, self.min_depth, self.max_depth))
                    continue

                # 2. Spike Check (Jump limit)
                if self.filtered_depth is not None:
                    if abs(raw_depth - self.filtered_depth) > self.max_diff:
                        rospy.logwarn_throttle(1, "Depth spike detected: %.2f (prev: %.2f)" % (raw_depth, self.filtered_depth))
                        continue
                # -------------------------

                # 3. Low Pass Filter (Exponential Moving Average)
                if self.filtered_depth is None:
                    self.filtered_depth = raw_depth
                else:
                    # y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
                    self.filtered_depth = self.filter_factor * raw_depth + (1.0 - self.filter_factor) * self.filtered_depth
                        
                # 4. Apply Calibration Offset (Tare)
                final_depth = self.filtered_depth - self.depth_offset

                # 5. Publish Depth
                depth_msg = PointStamped()
                depth_msg.header.stamp = rospy.Time.now()
                depth_msg.header.frame_id = "depth_sensor_link"
                depth_msg.point.z = -final_depth
                self.depth_pub.publish(depth_msg)
            else:
                rospy.logwarn("Sensor read failed!")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = MS5837Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
