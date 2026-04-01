#!/usr/bin/env python
import math
import numpy as np

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
    C = np.array([0.0, 0.8, 0.20])
    R = 0.10 # radius of the dome
    
    poses = []
    
    # 20 Grad bis 90 Grad Elevation, damit Z nicht in unsere Tisch-Toleranz (Z < 0.05m) faellt
    for elevation in [45, 70, 90]:
        phi = math.radians(elevation)
        # Oben an der Kugel ist nur 1 Punkt, an der Basis mehr
        num_points = 1 if elevation == 90 else (8 if elevation == 70 else 12)
        
        for i in range(num_points):
            theta = i * (2 * math.pi / num_points)
            
            # Position P
            x = C[0] + R * math.cos(phi) * math.cos(theta)
            y = C[1] + R * math.cos(phi) * math.sin(theta)
            z = C[2] + R * math.sin(phi)
            P = np.array([x, y, z])
            
            # Normalenvektor am Punkt P zeigt nach aussen
            n = (P - C) / R
            
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
            
            poses.append((x, y, z, rx, ry, rz))

    filename = "dome_poses.csv"
    with open(filename, 'w') as f:
        for p in poses:
            f.write("{:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}\n".format(*p))
            
    params_filename = "dome_params.csv"
    with open(params_filename, 'w') as f:
        f.write("{:.4f}, {:.4f}, {:.4f}, {:.4f}\n".format(C[0], C[1], C[2], R))
    
    print("===================")
    print("Erfolgreich {} Posen in '{}' generiert!".format(len(poses), filename))
    print("Mittelpunkt: [{}, {}, {}], Radius: {}m gespeichert in '{}'".format(C[0], C[1], C[2], R, params_filename))
    print("===================")

if __name__ == '__main__':
    main()