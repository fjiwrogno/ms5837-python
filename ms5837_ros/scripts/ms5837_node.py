#!/usr/bin/env python

import rospy
import ms5837
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PointStamped

def main():
    rospy.init_node('depth_sensor_node')

    # Parameters
    bus = rospy.get_param('~bus', 0)
    fluid_density = rospy.get_param('~fluid_density', 1000) # kg/m^3
    rate_hz = rospy.get_param('~rate', 100) # 建议提高频率，例如 100Hz

    # Initialize sensor
    try:
        sensor = ms5837.MS5837_30BA(bus)
        if not sensor.init():
            rospy.logerr("Sensor could not be initialized")
            return
        
        sensor.setFluidDensity(fluid_density)
    except Exception as e:
        rospy.logerr("Failed to initialize sensor: %s" % str(e))
        return

    # Publishers
    # pressure_pub = rospy.advertise('~pressure', FluidPressure, queue_size=1)
    depth_pub = rospy.advertise('~depth', PointStamped, queue_size=1)

    rate = rospy.Rate(rate_hz)

    # 定义两种模式的 OSR
    OSR_FAST = ms5837.OSR_1024  # ~5ms 延迟
    OSR_PRECISE = ms5837.OSR_8192 # ~41ms 延迟

    while not rospy.is_shutdown():
        current_osr = OSR_FAST
        if sensor.read(oversampling=current_osr):
            # # Pressure
            # pressure_msg = FluidPressure()
            # pressure_msg.header.stamp = rospy.Time.now()
            # pressure_msg.header.frame_id = "ms5837_link"
            # pressure_msg.fluid_pressure = sensor.pressure(ms5837.UNITS_Pa)
            # pressure_msg.variance = 0 # Unknown
            # pressure_pub.publish(pressure_msg)

            # Depth
            depth_msg = PointStamped()
            depth_msg.header.stamp = rospy.Time.now()
            depth_msg.header.frame_id = "depth_sensor_link"
            depth_msg.point.z = sensor.depth()
            depth_pub.publish(depth_msg)
        else:
            rospy.logwarn("Sensor read failed!")
        
        rate.sleep()

if __name__ == '__main__':
    main()
