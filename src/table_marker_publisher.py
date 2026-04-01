#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def publish_table_markers():
    rospy.init_node('table_marker_publisher')
    pub = rospy.Publisher('table_markers', MarkerArray, queue_size=10, latch=True)

    # Table bounds
    x_min, x_max = -0.38, 0.425
    y_min, y_max = -0.185, 1.02
    z = 0.0

    marker_array = MarkerArray()

    # 1. Plane marker (brown)
    plane = Marker()
    plane.header.frame_id = "base"
    plane.header.stamp = rospy.Time.now()
    plane.ns = "table"
    plane.id = 0
    plane.type = Marker.CUBE
    plane.action = Marker.ADD

    # Dimensions: X[-38, 42.5]=80.5cm, Y[-18.5, 102]=120.5cm, Z is virtually 0 thickness
    plane.scale.x = x_max - x_min
    plane.scale.y = y_max - y_min
    plane.scale.z = 0.001 

    # Center position
    plane.pose.position.x = (x_min + x_max) / 2.0
    plane.pose.position.y = (y_min + y_max) / 2.0
    plane.pose.position.z = z
    plane.pose.orientation.w = 1.0

    # Brown color
    plane.color.r = 0.54
    plane.color.g = 0.27
    plane.color.b = 0.07
    plane.color.a = 0.8  # slightly transparent
    
    marker_array.markers.append(plane)

    # 2. Corner Points (Spheres)
    corners = [
        (x_min, y_min),
        (x_max, y_min),
        (x_max, y_max),
        (x_min, y_max)
    ]

    for i, (cx, cy) in enumerate(corners):
        corner = Marker()
        corner.header.frame_id = "base"
        corner.header.stamp = rospy.Time.now()
        corner.ns = "table_corners"
        corner.id = i + 1
        corner.type = Marker.SPHERE
        corner.action = Marker.ADD

        corner.scale.x = 0.03
        corner.scale.y = 0.03
        corner.scale.z = 0.03

        corner.pose.position.x = cx
        corner.pose.position.y = cy
        corner.pose.position.z = z
        corner.pose.orientation.w = 1.0

        # Red color for corners
        corner.color.r = 1.0
        corner.color.g = 0.0
        corner.color.b = 0.0
        corner.color.a = 1.0

        marker_array.markers.append(corner)

    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        # Update timestamps
        now = rospy.Time.now()
        for m in marker_array.markers:
            m.header.stamp = now
        pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_table_markers()
    except rospy.ROSInterruptException:
        pass
