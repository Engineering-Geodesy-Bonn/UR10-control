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
        import tf
        self.listener = tf.TransformListener()
        self.pose_pub = rospy.Publisher('/target_poses', PoseArray, queue_size=10, latch=True)
        self.marker_pub = rospy.Publisher('/target_markers', MarkerArray, queue_size=10, latch=True)
        self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=10)
        self.current_tcp = None
        self.subscriber = rospy.Subscriber('/tool_data', ToolDataMsg, self.tool_data_callback)
        rospy.sleep(1.0) # give tf listener time to buffer

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
        """Fetch the current tool tip position from tf listener."""
        trans, _ = self.get_current_pose_full('/tool0_controller')
        return trans

    def get_current_pose_full(self, frame='/camera'):
        """Fetch current pose (trans, rot) for a frame in base coordinates."""
        import tf
        try:
            (trans, rot) = self.listener.lookupTransform('/base', frame, rospy.Time(0))
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

    def quat_to_ur_axis_angle(self, qx, qy, qz, qw):
        """ Convert quaternion to UR axis-angle representation (rx, ry, rz). """
        # Clamp w to [-1, 1] to handle numerical errors
        qw = max(-1.0, min(1.0, qw))
        theta = 2.0 * math.acos(qw)
        
        # Avoid division by zero
        sin_half_theta = math.sin(theta / 2.0)
        if abs(sin_half_theta) < 1e-6:
            return (0.0, 0.0, 0.0)
        
        rx = qx / sin_half_theta * theta
        ry = qy / sin_half_theta * theta
        rz = qz / sin_half_theta * theta
        return (rx, ry, rz)

    def rotation_distance(self, rx1, ry1, rz1, rx2, ry2, rz2):
        """
        Calculate shortest angular distance between two orientations.
        Input rotations are UR axis-angle vectors; output is radians in [0, pi].
        """
        q1x, q1y, q1z, q1w = self.ur_axis_angle_to_quat(rx1, ry1, rz1)
        q2x, q2y, q2z, q2w = self.ur_axis_angle_to_quat(rx2, ry2, rz2)

        dot = q1x * q2x + q1y * q2y + q1z * q2z + q1w * q2w
        dot = abs(dot)  # q and -q represent the same orientation
        dot = max(-1.0, min(1.0, dot))
        return 2.0 * math.acos(dot)

    def compute_pose_errors(self, x, y, z, rx, ry, rz, frame='/tool0_controller'):
        """Compute translation and shortest-path rotation error to target pose."""
        trans, rot = self.get_current_pose_full(frame)
        if not trans or not rot:
            return None

        target_x, target_y, target_z = x, y, z
        target_rx, target_ry, target_rz = rx, ry, rz

        # If camera-frame error is requested, convert commanded tool target pose
        # into expected camera target pose using the TF tree extrinsic only.
        if frame == '/camera':
            import tf
            import tf.transformations as tf_trans
            try:
                t_tc, q_tc = self.listener.lookupTransform('/tool0_controller', '/camera', rospy.Time(0))

                q_bt = self.ur_axis_angle_to_quat(rx, ry, rz)
                t_bt = tf_trans.quaternion_matrix(q_bt)
                t_bt[0, 3] = x
                t_bt[1, 3] = y
                t_bt[2, 3] = z

                t_tc_m = tf_trans.quaternion_matrix(q_tc)
                t_tc_m[0, 3] = t_tc[0]
                t_tc_m[1, 3] = t_tc[1]
                t_tc_m[2, 3] = t_tc[2]

                t_bc = tf_trans.concatenate_matrices(t_bt, t_tc_m)
                target_x = t_bc[0, 3]
                target_y = t_bc[1, 3]
                target_z = t_bc[2, 3]

                q_bc = tf_trans.quaternion_from_matrix(t_bc)
                target_rx, target_ry, target_rz = self.quat_to_ur_axis_angle(q_bc[0], q_bc[1], q_bc[2], q_bc[3])
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                return None

        dx = trans[0] - target_x
        dy = trans[1] - target_y
        dz = trans[2] - target_z
        trans_error = math.sqrt(dx**2 + dy**2 + dz**2)

        qx, qy, qz, qw = rot[0], rot[1], rot[2], rot[3]
        current_rx, current_ry, current_rz = self.quat_to_ur_axis_angle(qx, qy, qz, qw)
        rot_error = self.rotation_distance(current_rx, current_ry, current_rz, target_rx, target_ry, target_rz)

        return trans_error, rot_error, dx, dy, dz

    def convert_target_to_tool_pose(self, x, y, z, rx, ry, rz, target_frame='/tool0_controller'):
        """Convert a target pose from target_frame into /tool0_controller pose in /base."""
        if target_frame == '/tool0_controller':
            return x, y, z, rx, ry, rz

        if target_frame != '/camera':
            rospy.logerr("Unsupported target frame: {}. Use /tool0_controller or /camera".format(target_frame))
            return None

        import tf
        import tf.transformations as tf_trans
        try:
            # Known extrinsic from tool to camera from TF tree.
            t_tc, q_tc = self.listener.lookupTransform('/tool0_controller', '/camera', rospy.Time(0))

            # Build target transform T_base_target (here: T_base_camera)
            q_bt = self.ur_axis_angle_to_quat(rx, ry, rz)
            t_bt = tf_trans.quaternion_matrix(q_bt)
            t_bt[0, 3] = x
            t_bt[1, 3] = y
            t_bt[2, 3] = z

            # Build T_tool_camera and invert it to get T_camera_tool
            t_tc_m = tf_trans.quaternion_matrix(q_tc)
            t_tc_m[0, 3] = t_tc[0]
            t_tc_m[1, 3] = t_tc[1]
            t_tc_m[2, 3] = t_tc[2]
            t_ct_m = tf_trans.inverse_matrix(t_tc_m)

            # T_base_tool = T_base_camera * T_camera_tool
            t_btool = tf_trans.concatenate_matrices(t_bt, t_ct_m)
            q_btool = tf_trans.quaternion_from_matrix(t_btool)
            rx_tool, ry_tool, rz_tool = self.quat_to_ur_axis_angle(q_btool[0], q_btool[1], q_btool[2], q_btool[3])

            return t_btool[0, 3], t_btool[1, 3], t_btool[2, 3], rx_tool, ry_tool, rz_tool
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logerr("Failed to convert target pose from {} to /tool0_controller: {}".format(target_frame, e))
            return None

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

    def move_and_wait(self, x, y, z, rx, ry, rz, a, v, is_first_point=False, wait_timeout=60.0):
        if not self.check_bounds(x, y, z):
            return False, None

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
        timeout = rospy.Time.now() + rospy.Duration(wait_timeout)
        success = False
        translation_tolerance = 0.005  # 5mm tolerance for translation
        rotation_tolerance = math.radians(1.0)  # 1 degree tolerance for rotation (in radians)
        trans_error = None
        rot_error = None
        dx_error = None
        dy_error = None
        dz_error = None

        # Check loop
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            trans, rot = self.get_current_pose_full('/tool0_controller')
            if trans and rot:
                # Check translation
                dx = trans[0] - x
                dy = trans[1] - y
                dz = trans[2] - z
                trans_dist = math.sqrt(dx**2 + dy**2 + dz**2)

                # Check rotation - convert current quaternion to UR axis-angle format
                qx, qy, qz, qw = rot[0], rot[1], rot[2], rot[3]
                current_rx, current_ry, current_rz = self.quat_to_ur_axis_angle(qx, qy, qz, qw)
                rot_dist = self.rotation_distance(current_rx, current_ry, current_rz, rx, ry, rz)

                if trans_dist < translation_tolerance and rot_dist < rotation_tolerance:
                    # Give it a fraction of a second to settle
                    rospy.sleep(0.5)
                    # Check again to ensure it really stopped (both translation and rotation)
                    trans_settled, rot_settled = self.get_current_pose_full('/tool0_controller')
                    if trans_settled and rot_settled:
                        dx_settled = trans_settled[0] - x
                        dy_settled = trans_settled[1] - y
                        dz_settled = trans_settled[2] - z
                        trans_dist_settled = math.sqrt(dx_settled**2 + dy_settled**2 + dz_settled**2)
                        
                        qx_s, qy_s, qz_s, qw_s = rot_settled[0], rot_settled[1], rot_settled[2], rot_settled[3]
                        current_rx_s, current_ry_s, current_rz_s = self.quat_to_ur_axis_angle(qx_s, qy_s, qz_s, qw_s)
                        rot_dist_settled = self.rotation_distance(current_rx_s, current_ry_s, current_rz_s, rx, ry, rz)
                        
                        if trans_dist_settled < translation_tolerance and rot_dist_settled < rotation_tolerance:
                            success = True
                            trans_error = trans_dist_settled
                            rot_error = rot_dist_settled
                            dx_error = dx_settled
                            dy_error = dy_settled
                            dz_error = dz_settled
                            rospy.loginfo("Target reached! Translation error: {:.6f}m, Rotation error: {:.6f}rad".format(trans_dist_settled, rot_dist_settled))
                            break
            
            rate.sleep()

        # If tolerance was not reached before timeout, still report best-effort error
        # from the current pose so downstream camera-trigger logging can print metrics.
        if not success:
            trans_final, rot_final = self.get_current_pose_full('/tool0_controller')
            if trans_final and rot_final:
                dx_error = trans_final[0] - x
                dy_error = trans_final[1] - y
                dz_error = trans_final[2] - z
                trans_error = math.sqrt(dx_error**2 + dy_error**2 + dz_error**2)

                qx_f, qy_f, qz_f, qw_f = rot_final[0], rot_final[1], rot_final[2], rot_final[3]
                current_rx_f, current_ry_f, current_rz_f = self.quat_to_ur_axis_angle(qx_f, qy_f, qz_f, qw_f)
                rot_error = self.rotation_distance(current_rx_f, current_ry_f, current_rz_f, rx, ry, rz)
        
        return success, (trans_error, rot_error, dx_error, dy_error, dz_error)
def main():
    # Initialize ROS node VERY EARLY before argument parsing / logging
    rospy.init_node('move_to_xyz_node', anonymous=True)

    parser = argparse.ArgumentParser(description="Move arm sequentially through XYZ coordinates in a CSV.")
    parser.add_argument("--csv", type=str, default="dome_poses.csv", help="Path to CSV file with XYZ coordinates")
    
    # Optional arguments for orientation
    parser.add_argument("--rx", type=float, default=3.14, help="Orientation Rx (default: 3.14)")
    parser.add_argument("--ry", type=float, default=0.0, help="Orientation Ry (default: 0.0)")
    parser.add_argument("--rz", type=float, default=0.0, help="Orientation Rz (default: 0.0)")
    parser.add_argument("--target-frame", type=str, default="camera",
                        help="Frame of input target poses in CSV: tool0_controller or camera (default: camera)")
    parser.add_argument("--error-frame", type=str, default="tool0_controller",
                        help="TF frame for error evaluation (default: tool0_controller)")
    parser.add_argument("--wait-timeout", type=float, default=60.0,
                        help="Max seconds to wait for reaching pose tolerance (default: 60.0)")
    
    # Optional arguments for dynamics
    parser.add_argument("-a", "--acc", type=float, default=0.5, help="Joint acceleration")
    parser.add_argument("-v", "--vel", type=float, default=0.2, help="Joint velocity")

    args = parser.parse_args(rospy.myargv()[1:])
    target_frame = args.target_frame if args.target_frame.startswith('/') else '/' + args.target_frame
    error_frame = args.error_frame if args.error_frame.startswith('/') else '/' + args.error_frame

    if target_frame == '/camera' and error_frame == '/tool0_controller':
        error_frame = '/camera'
        rospy.loginfo("Auto-setting error frame to /camera because target frame is /camera")

    if target_frame not in ['/tool0_controller', '/camera']:
        rospy.logerr("Invalid --target-frame '{}'. Use tool0_controller or camera.".format(args.target_frame))
        return

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
    host_home = os.environ.get('HOST_HOME', os.path.expanduser('~'))
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
    csv_writer.writerow([
        'filename',
        'camera_x_mm', 'camera_y_mm', 'camera_z_mm',
        'camera_rx_rad', 'camera_ry_rad', 'camera_rz_rad',
        'camera_qx', 'camera_qy', 'camera_qz', 'camera_qw'
    ])

    # Publish markers (white spheres and optional dome)
    ur_control.publish_markers(targets, dome_params)

    # Publish and print them as PoseArray (optional, good for debugging)
    ur_control.publish_poses(targets)

    for i, t in enumerate(targets):
        target_for_motion = ur_control.convert_target_to_tool_pose(
            t['x'], t['y'], t['z'], t['rx'], t['ry'], t['rz'], target_frame
        )
        if target_for_motion is None:
            rospy.logwarn("Skipping point {}/{} due to TF conversion failure.".format(i+1, len(targets)))
            continue

        cmd_x, cmd_y, cmd_z, cmd_rx, cmd_ry, cmd_rz = target_for_motion

        rospy.loginfo("--- Moving to Point {}/{} ---".format(i+1, len(targets)))
        rospy.loginfo("Target ({}) : X={}, Y={}, Z={}, Rx={}, Ry={}, Rz={}, a={}, v={}".format(
            target_frame, t['x'], t['y'], t['z'], t['rx'], t['ry'], t['rz'], t['a'], t['v']
        ))
        rospy.loginfo("Commanded tool pose: X={}, Y={}, Z={}, Rx={}, Ry={}, Rz={}".format(
            cmd_x, cmd_y, cmd_z, cmd_rx, cmd_ry, cmd_rz
        ))
        
        # Use movej for the first point to safely reach the dome, then movel to stay in the correct configuration
        is_first = (i == 0)
        success, _ = ur_control.move_and_wait(
            cmd_x, cmd_y, cmd_z, cmd_rx, cmd_ry, cmd_rz, t['a'], t['v'],
            is_first_point=is_first, wait_timeout=args.wait_timeout
        )
        
        if not rospy.is_shutdown():
            if success:
                rospy.loginfo("Reached position. Waiting 1 second before triggering camera...")
            else:
                rospy.logwarn("Motion wait timeout before tolerance. Triggering camera with current pose/error values.")
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
                    # Print error information at camera trigger time using fresh TF data
                    trigger_error_data = ur_control.compute_pose_errors(
                        t['x'], t['y'], t['z'], t['rx'], t['ry'], t['rz'], error_frame
                    )
                    if trigger_error_data and all(v is not None for v in trigger_error_data):
                        trans_error, rot_error, dx_error, dy_error, dz_error = [float(v) for v in trigger_error_data]
                        print("\n" + "="*70)
                        print("CAMERA TRIGGERED - Point {}/{}".format(i+1, len(targets)))
                        print("="*70)
                        print("Error Frame: {}".format(error_frame))
                        print("Translational Error: {:.6f}m".format(trans_error))
                        print("  dX: {:.6f}m, dY: {:.6f}m, dZ: {:.6f}m".format(
                            dx_error, dy_error, dz_error))
                        print("Rotational Error: {:.6f} rad ({:.4f} deg)".format(
                            rot_error, math.degrees(rot_error)))
                        print("="*70 + "\n")
                    else:
                        rospy.logwarn("Camera triggered but no valid numeric error data available for this point.")
                    
                    rospy.loginfo("Camera triggered successfully. Image: " + img_filename)
                    # Grab true current camera pose for CSV
                    trans, rot = ur_control.get_current_pose_full('/camera')
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