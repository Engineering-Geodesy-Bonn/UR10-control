#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import rospy
import math
import os
import csv
import rospkg
import sys
import datetime
from std_msgs.msg import String
from std_srvs.srv import Trigger

if sys.version_info[0] < 3:
    input_func = raw_input
else:
    input_func = input
from ur_msgs.msg import ToolDataMsg
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray

# Table bounding box (in meters relative to base)
# X: [-0.38, 0.425], Y: [-0.185, 1.020]
MIN_X = -0.38
MAX_X = 0.425
MIN_Y = 0.300
MAX_Y = 1.020
MIN_Z = 0.050  # minimum height above table (safety margin 5cm)
MAX_Z = 1.200   # arbitrary maximum height

class URController:
    def __init__(self):
        self.pose_pub = rospy.Publisher('/target_poses', PoseArray, queue_size=10, latch=True)
        self.marker_pub = rospy.Publisher('/target_markers', MarkerArray, queue_size=10, latch=True)
        self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=10)
        self.current_tcp = None
        self.subscriber = rospy.Subscriber('/tool_data', ToolDataMsg, self.tool_data_callback)
        rospy.sleep(0.5)

    def tool_data_callback(self, msg):
        self.current_tcp = msg

    def check_bounds(self, x, y, z):
        if not (MIN_X <= x <= MAX_X):
            rospy.logerr("X coordinate ({}) out of bounds [{}, {}]!".format(x, MIN_X, MAX_X))
            return False
        if not (MIN_Y <= y <= MAX_Y):
            rospy.logerr("Y coordinate ({}) out of bounds [{}, {}]!".format(y, MIN_Y, MAX_Y))
            return False
        if not (MIN_Z <= z <= MAX_Z):
            rospy.logerr("Z coordinate ({}) out of bounds [{}, {}]!".format(z, MIN_Z, MAX_Z))
            return False
        return True

    def get_current_pose(self):
        """ Fetch the current tool tip pose from tf listener. """
        import tf
        listener = tf.TransformListener()
        try:
            listener.waitForTransform('/base', '/tool0_controller', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
            return trans
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            return None

    def get_current_pose_full(self):
        """ Fetch the full current tool tip pose (trans, rot) from tf listener. """
        import tf
        listener = tf.TransformListener()
        try:
            listener.waitForTransform('/base', '/tool0_controller', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
            return trans, rot
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            return None, None

    def ur_axis_angle_to_quat(self, rx, ry, rz):
        theta = math.sqrt(rx**2 + ry**2 + rz**2)
        if theta < 1e-6:
            return (0.0, 0.0, 0.0, 1.0)
        s = math.sin(theta / 2.0)
        x = (rx / theta) * s
        y = (ry / theta) * s
        z = (rz / theta) * s
        w = math.cos(theta / 2.0)
        return (x, y, z, w)

    def publish_poses(self, targets):
        pose_array = PoseArray()
        pose_array.header.frame_id = "base"
        pose_array.header.stamp = rospy.Time.now()
        
        print("\n--- Target Poses Loaded ---")
        for i, t in enumerate(targets):
            x, y, z = t['x'], t['y'], t['z']
            rx, ry, rz = t['rx'], t['ry'], t['rz']
            
            qx, qy, qz, qw = self.ur_axis_angle_to_quat(rx, ry, rz)
            
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            
            pose_array.poses.append(pose)
            print("Waypoint {}: Pos [X: {:.3f}, Y: {:.3f}, Z: {:.3f}], Orient_UR [Rx: {:.3f}, Ry: {:.3f}, Rz: {:.3f}], Quat [x: {:.3f}, y: {:.3f}, z: {:.3f}, w: {:.3f}]".format(
                i+1, x, y, z, rx, ry, rz, qx, qy, qz, qw))
        print("---------------------------\n")    
            
        self.pose_pub.publish(pose_array)

    def publish_markers(self, targets, dome_params=None):
        marker_array = MarkerArray()
        
        # Publish white spheres for targets
        for i, t in enumerate(targets):
            x, y, z = t['x'], t['y'], t['z']
            marker = Marker()
            marker.header.frame_id = "base"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "csv_targets"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            
            # White markers
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)

        # Publish the dome if params are provided
        if dome_params:
            cx, cy, cz, radius = dome_params
            dome = Marker()
            dome.header.frame_id = "base"
            dome.header.stamp = rospy.Time.now()
            dome.ns = "dome"
            dome.id = 9999
            dome.type = Marker.SPHERE
            dome.action = Marker.ADD
            dome.pose.position.x = cx
            dome.pose.position.y = cy
            dome.pose.position.z = cz
            dome.pose.orientation.w = 1.0
            
            # Scale is diameter
            dome.scale.x = radius * 2
            dome.scale.y = radius * 2
            dome.scale.z = radius * 2
            
            # Light white / transparent
            dome.color.r = 1.0
            dome.color.g = 1.0
            dome.color.b = 1.0
            dome.color.a = 0.3
            
            marker_array.markers.append(dome)
            
        self.marker_pub.publish(marker_array)

    def move_and_wait(self, x, y, z, rx, ry, rz, a, v, is_first_point=False):
        if not self.check_bounds(x, y, z):
            return False

        # Send URScript command. 
        # Wir hatten anfangs "movej" fuer ALLES verwendet. Das war das Setup,
        # als deine Schulter noch "besser" war (bevor ich "is_first_point" eingebaut habe).
        # Wir reverting to pure movej for everything, without get_inverse_kin, 
        # basically identical to what we had in the "good" state.
        ur_script = "movej(p[{:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}], a={:.2f}, v={:.2f})\n".format(
            x, y, z, rx, ry, rz, a, v
        )
        
        rospy.loginfo("Sending command: " + ur_script.strip())
        self.pub.publish(ur_script)

        # Wait for the motion to complete by tracking TCP pose
        rospy.loginfo("Waiting for arm to reach target...")
        timeout = rospy.Time.now() + rospy.Duration(15.0)  # max wait 15 seconds
        success = False
        target_tolerance = 0.005 # 5mm tolerance

        # Check loop
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            current_pos = self.get_current_pose()
            if current_pos:
                dx = current_pos[0] - x
                dy = current_pos[1] - y
                dz = current_pos[2] - z
                dist = math.sqrt(dx**2 + dy**2 + dz**2)

                if dist < target_tolerance:
                    # Give it a fraction of a second to settle
                    rospy.sleep(0.5)
                    # Check again to ensure it really stopped
                    current_pos_settled = self.get_current_pose()
                    if current_pos_settled:
                        dist_settled = math.sqrt((current_pos_settled[0]-x)**2 + (current_pos_settled[1]-y)**2 + (current_pos_settled[2]-z)**2)
                        if dist_settled < target_tolerance:
                            success = True
                            break
            rate.sleep()
        
        return success
def main():
    # Initialize ROS node VERY EARLY before argument parsing / logging
    rospy.init_node('move_to_xyz_node', anonymous=True)

    parser = argparse.ArgumentParser(description="Move arm sequentially through XYZ coordinates in a CSV.")
    parser.add_argument("--csv", type=str, default="dome_poses.csv", help="Path to CSV file with XYZ coordinates")
    
    # Optional arguments for orientation
    parser.add_argument("--rx", type=float, default=3.14, help="Orientation Rx (default: 3.14)")
    parser.add_argument("--ry", type=float, default=0.0, help="Orientation Ry (default: 0.0)")
    parser.add_argument("--rz", type=float, default=0.0, help="Orientation Rz (default: 0.0)")
    
    # Optional arguments for dynamics
    parser.add_argument("-a", "--acc", type=float, default=0.5, help="Joint acceleration")
    parser.add_argument("-v", "--vel", type=float, default=0.2, help="Joint velocity")

    args = parser.parse_args(rospy.myargv()[1:])

    # Find the file either at the absolute path specified, or relative to the package dir
    csv_path = args.csv
    if not os.path.exists(csv_path):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('ur_modern_driver')
        csv_path = os.path.join(pkg_path, args.csv)

    if not os.path.exists(csv_path):
        rospy.logerr("File not found: {}".format(csv_path))
        rospy.loginfo("Please create {} with lines formatted like: X, Y, Z".format(args.csv))
        return

    targets = []
    # targets list will store dicts: {'x':.., 'y':.., 'z':.., 'rx':.., 'ry':.., 'rz':.., 'a':.., 'v':..}
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row: continue
            try:
                # Basic format: X, Y, Z
                # Extended format: X, Y, Z, Rx, Ry, Rz, Acc, Vel
                if len(row) >= 3:
                    t = {
                        'x': float(row[0]), 'y': float(row[1]), 'z': float(row[2]),
                        'rx': args.rx, 'ry': args.ry, 'rz': args.rz,
                        'a': args.acc, 'v': args.vel
                    }
                    if len(row) >= 6:
                        t['rx'] = float(row[3])
                        t['ry'] = float(row[4])
                        t['rz'] = float(row[5])
                    if len(row) >= 8:
                        t['a'] = float(row[6])
                        t['v'] = float(row[7])
                        
                    targets.append(t)
            except ValueError:
                pass # skip header or invalid rows

    if not targets:
        rospy.logwarn("No valid coordinates found in the CSV.")
        return

    # Check for dome parameters
    dome_params = None
    dome_csv_path = os.path.join(os.path.dirname(csv_path), "dome_params.csv")
    if os.path.exists(dome_csv_path):
        try:
            with open(dome_csv_path, 'r') as f:
                r = csv.reader(f)
                for row in r:
                    if len(row) >= 4:
                        dome_params = (float(row[0]), float(row[1]), float(row[2]), float(row[3]))
                        break
        except Exception:
            pass

    ur_control = URController()
    
    # Create the run folder mapped to /home/felix/Pictures/UR10arm on host
    host_home = os.environ.get('HOST_HOME', '/root')
    pictures_dir = os.path.join(host_home, 'Pictures', 'UR10arm')
    now = datetime.datetime.utcnow()
    # Format: YYYY-MM-DD_HH-MM-SS
    run_folder_name = now.strftime("%Y-%m-%d_%H-%M-%S")
    run_dir = os.path.join(pictures_dir, run_folder_name)
    if not os.path.exists(run_dir):
        os.makedirs(run_dir)
        
    csv_out_path = os.path.join(run_dir, "poses.csv")
    csv_file = open(csv_out_path, 'w')
    csv_writer = csv.writer(csv_file)
    # Header
    csv_writer.writerow(['filename', 'x_mm', 'y_mm', 'z_mm', 'rx_rad', 'ry_rad', 'rz_rad', 'qx', 'qy', 'qz', 'qw'])

    # Publish markers (white spheres and optional dome)
    ur_control.publish_markers(targets, dome_params)

    # Publish and print them as PoseArray (optional, good for debugging)
    ur_control.publish_poses(targets)

    for i, t in enumerate(targets):
        rospy.loginfo("--- Moving to Point {}/{} ---".format(i+1, len(targets)))
        rospy.loginfo("Target: X={}, Y={}, Z={}, Rx={}, Ry={}, Rz={}, a={}, v={}".format(
            t['x'], t['y'], t['z'], t['rx'], t['ry'], t['rz'], t['a'], t['v']
        ))
        
        # Use movej for the first point to safely reach the dome, then movel to stay in the correct configuration
        is_first = (i == 0)
        success = ur_control.move_and_wait(t['x'], t['y'], t['z'], t['rx'], t['ry'], t['rz'], t['a'], t['v'], is_first_point=is_first)
        
        if not rospy.is_shutdown():
            rospy.loginfo("Reached position. Waiting 1 second before triggering camera...")
            rospy.sleep(1.0)
            
            # Format time HH:MM:SS.sss
            now_trigger = datetime.datetime.utcnow()
            img_filename = now_trigger.strftime("%H:%M:%S.%f")[:-3] + ".png"
            img_save_path = os.path.join(run_dir, img_filename)
            
            # Tell trigger camera node where to save the image
            rospy.set_param('/trigger_camera/save_path', img_save_path)
            
            try:
                rospy.wait_for_service('/trigger_camera', timeout=2.0)
                trigger_service = rospy.ServiceProxy('/trigger_camera', Trigger)
                resp = trigger_service()
                
                if resp.success:
                    rospy.loginfo("Camera triggered successfully. Image: " + img_filename)
                    # Grab true current pose for CSV
                    trans, rot = ur_control.get_current_pose_full()
                    if trans and rot:
                        x_mm, y_mm, z_mm = trans[0]*1000.0, trans[1]*1000.0, trans[2]*1000.0
                        qx, qy, qz, qw = rot[0], rot[1], rot[2], rot[3]
                        import tf.transformations as tf_trans
                        rx, ry, rz = tf_trans.euler_from_quaternion([qx, qy, qz, qw])
                        # Write row (translation mm, angles 9 decimal places rad)
                        csv_writer.writerow([
                            img_filename, 
                            "%.3f" % x_mm, "%.3f" % y_mm, "%.3f" % z_mm,
                            "%.9f" % rx, "%.9f" % ry, "%.9f" % rz,
                            "%.9f" % qx, "%.9f" % qy, "%.9f" % qz, "%.9f" % qw
                        ])
                        csv_file.flush()
                else:
                    rospy.logwarn("Failed to trigger camera: " + resp.message)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
            except rospy.ROSException:
                rospy.logerr("Service /trigger_camera not available.")
            
            rospy.loginfo("Waiting 1 second before continuing...")
            rospy.sleep(1.0)
            
    csv_file.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass