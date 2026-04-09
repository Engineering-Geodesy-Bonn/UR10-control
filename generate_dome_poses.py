#!/usr/bin/env python

# Copy new generated poses to docker container fancy-arm:
#docker cp dome_poses.csv fancy-arm:/catkin_ws/src/ur_modern_driver/dome_poses.csv
#docker cp dome_params.csv fancy-arm:/catkin_ws/src/ur_modern_driver/dome_params.csv


import math
import os
import argparse
import numpy as np


def plot_dome_and_poses(center, radius, poses, triad_scale=0.02, plot_file="dome_poses_plot.png"):
    try:
        import matplotlib
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
    except ImportError:
        print("[WARN] matplotlib not installed. Skipping 3D plot.")
        return

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Sphere mesh
    u = np.linspace(0, 2 * math.pi, 60)
    v = np.linspace(0, math.pi, 30)
    xs = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    ys = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    zs = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(xs, ys, zs, alpha=0.15, color='gray', linewidth=0)

    # Pose points + coordinate triads
    for pose in poses:
        pos = pose['position']
        rot = pose['R']

        ax.scatter(pos[0], pos[1], pos[2], color='k', s=18)
        ax.quiver(pos[0], pos[1], pos[2], rot[0, 0], rot[1, 0], rot[2, 0],
                  length=triad_scale, normalize=True, color='r')
        ax.quiver(pos[0], pos[1], pos[2], rot[0, 1], rot[1, 1], rot[2, 1],
                  length=triad_scale, normalize=True, color='g')
        ax.quiver(pos[0], pos[1], pos[2], rot[0, 2], rot[1, 2], rot[2, 2],
                  length=triad_scale, normalize=True, color='b')

    ax.scatter(center[0], center[1], center[2], color='m', s=40, label='Sphere center')
    ax.set_title('Dome poses with local coordinate triads')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass
    ax.legend()
    plt.tight_layout()

    backend = matplotlib.get_backend().lower()
    non_interactive_backends = {"agg", "cairo", "pdf", "pgf", "ps", "svg", "template"}
    if backend in non_interactive_backends:
        fig.savefig(plot_file, dpi=200)
        print("[INFO] Non-GUI matplotlib backend '{}' detected. Plot saved to '{}'".format(
            backend, os.path.abspath(plot_file)))
        plt.close(fig)
    else:
        plt.show()

def mat2quat(orig_M):
    M = np.array(orig_M)
    Qxx, Qyx, Qzx = M[0,0], M[1,0], M[2,0]
    Qxy, Qyy, Qzy = M[0,1], M[1,1], M[2,1]
    Qxz, Qyz, Qzz = M[0,2], M[1,2], M[2,2]
    t = Qxx + Qyy + Qzz
    if t > 0:
        r = math.sqrt(1 + t)
        w = 0.5 * r
        r = 0.5 / r
        x = (Qzy - Qyz) * r
        y = (Qxz - Qzx) * r
        z = (Qyx - Qxy) * r
    else:
        if Qxx > Qyy and Qxx > Qzz:
            r = math.sqrt(1 + Qxx - Qyy - Qzz)
            x = 0.5 * r
            r = 0.5 / r
            y = (Qyx + Qxy) * r
            z = (Qxz + Qzx) * r
            w = (Qzy - Qyz) * r
        elif Qyy > Qzz:
            r = math.sqrt(1 + Qyy - Qxx - Qzz)
            y = 0.5 * r
            r = 0.5 / r
            x = (Qyx + Qxy) * r
            z = (Qzy + Qyz) * r
            w = (Qxz - Qzx) * r
        else:
            r = math.sqrt(1 + Qzz - Qxx - Qyy)
            z = 0.5 * r
            r = 0.5 / r
            x = (Qxz + Qzx) * r
            y = (Qzy + Qyz) * r
            w = (Qyx - Qxy) * r
    return float(x), float(y), float(z), float(w)

def quat2axisangle(x, y, z, w):
    norm = math.sqrt(x*x + y*y + z*z)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0
    w_clip = max(-1.0, min(1.0, w))
    angle = 2 * math.acos(w_clip)
    return (x/norm * angle), (y/norm * angle), (z/norm * angle)

def main():
    parser = argparse.ArgumentParser(description="Generate dome poses with orthogonal offset from sphere surface")
    parser.add_argument("--offset", type=float, default=0.08,
                        help="Orthogonal offset from sphere surface in meters (positive = outside, negative = inside)")
    parser.add_argument("--azimuth-step-deg", type=float, default=None,
                        help="Azimuth step in degrees for each latitude ring (overrides default ring resolution)")
    parser.add_argument("--points-per-ring", type=int, default=None,
                        help="Fixed number of points per ring (except top point), overrides azimuth-step-deg")
    parser.add_argument("--no-plot", action="store_true",
                        help="Disable 3D visualization plot")
    parser.add_argument("--plot-file", type=str, default="dome_poses_plot.png",
                        help="Output image file used when no GUI backend is available")
    args = parser.parse_args()

    C = np.array([0.0, 0.7, 0.25])
    R = 0.10 # radius of the dome
    offset = args.offset
    
    poses = []
    
    # 20 Grad bis 90 Grad Elevation, damit Z nicht in unsere Tisch-Toleranz (Z < 0.05m) faellt
    for elevation in [20, 30, 40, 50, 60, 70, 90]:
        phi = math.radians(elevation)
        # Oben an der Kugel ist nur 1 Punkt, an der Basis mehr
        if elevation == 90:
            num_points = 1
        elif args.points_per_ring is not None:
            num_points = max(3, args.points_per_ring)
        elif args.azimuth_step_deg is not None:
            step_deg = max(0.1, float(args.azimuth_step_deg))
            num_points = max(3, int(round(360.0 / step_deg)))
        else:
            num_points = 8 if elevation == 70 else 12
        
        for i in range(num_points):
            theta = i * (2 * math.pi / num_points)
            
            # Surface point on sphere
            sx = C[0] + R * math.cos(phi) * math.cos(theta)
            sy = C[1] + R * math.cos(phi) * math.sin(theta)
            sz = C[2] + R * math.sin(phi)
            S = np.array([sx, sy, sz])

            # Outward normal at surface point
            n = (S - C) / R

            # Final pose position with orthogonal offset from sphere surface
            P = S + offset * n
            x, y, z = P[0], P[1], P[2]
            
            # Tool Z Achse schaut in die Kugel auf der Oberflaeche
            z_t = -n
            
            # Up-Vektor im Weltkoordinatensystem
            up = np.array([0.0, 0.0, 1.0])
            
            # Tool X Achse soll nach Norden (oben +Z) zeigen. Das machen wir
            # durch eine Projektion des up-Vektors auf die tangentiale Ebene.
            x_t = up - np.dot(up, n) * n
            x_t_norm = np.linalg.norm(x_t)
            
            # Sonderfall komplett oben (Nordpol)
            if x_t_norm > 1e-6:
                x_t = x_t / x_t_norm
            else:
                x_t = np.array([1.0, 0.0, 0.0])
                
            # Tool Y Achse via Kreuzprodukt (Right-Hand-Rule)
            y_t = np.cross(z_t, x_t)
            
            # Erzeuge Rotationsmatrix
            R_mat = np.column_stack((x_t, y_t, z_t))
            
            # Konvertiere von Matrix zu Quaternion -> dann zu AxisAngle Format (Rx,Ry,Rz) des Roboters
            qx, qy, qz, qw = mat2quat(R_mat)
            rx, ry, rz = quat2axisangle(qx, qy, qz, qw)
            
            poses.append({
                'csv': (x, y, z, rx, ry, rz),
                'position': P,
                'R': R_mat
            })

    filename = "dome_poses.csv"
    with open(filename, 'w') as f:
        for p in poses:
            f.write("{:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}\n".format(*p['csv']))
            
    params_filename = "dome_params.csv"
    with open(params_filename, 'w') as f:
        f.write("{:.4f}, {:.4f}, {:.4f}, {:.4f}\n".format(C[0], C[1], C[2], R))
    
    print("===================")
    print("Successfully created {} poses in '{}' !".format(len(poses), filename))
    print("Center: [{}, {}, {}], Radius: {}m strored in '{}'".format(C[0], C[1], C[2], R, params_filename))
    print("Orthogonal offset to sphere surface: {:.4f} m".format(offset))
    if args.points_per_ring is not None:
        print("Azimuth resolution: fixed points per ring = {}".format(max(3, args.points_per_ring)))
    elif args.azimuth_step_deg is not None:
        print("Azimuth resolution: step = {:.3f} deg".format(max(0.1, float(args.azimuth_step_deg))))
    else:
        print("Azimuth resolution: default ring-specific values (12 / 8 / 1)")
    print("===================")

    if not args.no_plot:
        plot_dome_and_poses(C, R, poses, plot_file=args.plot_file)

if __name__ == '__main__':
    main()