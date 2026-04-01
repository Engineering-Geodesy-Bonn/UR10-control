#!/usr/bin/env python
import os
import cv2
import rospy
import actionlib
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Trigger, TriggerResponse
from camera_control_msgs.msg import GrabImagesAction, GrabImagesGoal
from sensor_msgs.msg import Image

pub = None
client = None
bridge = CvBridge()

def handle_trigger(req):
    global pub, client, bridge
    rospy.loginfo("Trigger received! Grabbing image...")
    goal = GrabImagesGoal()
    # To grab an image via action we must provide at least one target parameter
    # so the driver knows how many images to grab. We request 1 image.
    goal.exposure_given = True
    goal.exposure_times = [5000.0] # default fallback exposure in microseconds
    goal.gain_given = False
    goal.gamma_given = False
    goal.brightness_given = False

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(5.0))
    result = client.get_result()

    if result and result.success and len(result.images) > 0:
        img_msg = result.images[0]
        pub.publish(img_msg)
        rospy.loginfo("Image grabbed and published to /camera_trigger/image_raw")
        
        # Save to host home directory
        try:
            cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            host_home = os.environ.get('HOST_HOME', '/root')
            save_path = os.path.join(host_home, 'basler_image.png')
            cv2.imwrite(save_path, cv_img)
            rospy.loginfo("Image successfully saved as .png to: " + save_path)
            return TriggerResponse(success=True, message="Image grabbed and saved to " + save_path)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: " + str(e))
            return TriggerResponse(success=True, message="Image grabbed but failed to save as PNG")
    else:
        rospy.logerr("Failed to grab image")
        return TriggerResponse(success=False, message="Failed to grab image")

def trigger_server():
    global pub, client
    rospy.init_node('camera_trigger_node')
    pub = rospy.Publisher('/camera_trigger/image_raw', Image, queue_size=1)
    
    rospy.loginfo("Waiting for GrabImages action server '/pylon_camera_node/grab_images_raw'...")
    client = actionlib.SimpleActionClient('/pylon_camera_node/grab_images_raw', GrabImagesAction)
    
    # Wait up to 10 seconds for the action server
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("Action server not available! Ensure pylon_camera_node is running.")
        return

    rospy.loginfo("Action server found. Starting trigger service...")
    s = rospy.Service('/trigger_camera', Trigger, handle_trigger)
    rospy.loginfo("Ready to trigger camera via rosservice call /trigger_camera")
    
    rospy.spin()

if __name__ == '__main__':
    trigger_server()
