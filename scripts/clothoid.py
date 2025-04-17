import numpy as np
import matplotlib.pyplot as plt
import math
import time
import rclpy

from clothoid_arc_turn.dual_scene import solveDualScene
from clothoid_arc_turn.scene import Scene

from drive import DriveVehicle

from rrt import RRT
from helper_functions import *


class MotionModel(DriveVehicle):
    def __init__(self):
        super().__init__(self)

    def drive(self, linear_vel, angular_vel):
        self.publish_steer_angle(angular_vel)
        self.publish_cmd_vel(linear_vel)
    
    def generate_time_aligned_commands(self, dense_paths, speed=1.0, wheelbase=2.5, steer_limit=0.4):
        commands = []
        for path in dense_paths:
            for i in range(1, len(path) - 1):
                p0, p1, p2 = path[i - 1], path[i], path[i + 1]
                ds = np.linalg.norm(p2[:2] - p0[:2]) / 2
                dtheta = p2[2] - p0[2]
                while dtheta > math.pi: dtheta -= 2 * math.pi
                while dtheta < -math.pi: dtheta += 2 * math.pi

                curvature = dtheta / ds if ds != 0 else 0
                delta = math.atan(wheelbase * curvature)
                delta = np.clip(delta, -steer_limit, steer_limit)
                commands.append((speed, delta))
        return commands

    def densify_path(self, path, spacing=0.1):
        from scipy.interpolate import interp1d

        # Cumulative arc-length
        points = path[:, :2]
        diffs = np.diff(points, axis=0)
        dists = np.hypot(diffs[:, 0], diffs[:, 1])
        cumdist = np.insert(np.cumsum(dists), 0, 0)

        total_length = cumdist[-1]
        new_s = np.arange(0, total_length, spacing)

        # Interpolate x, y, theta
        fx = interp1d(cumdist, path[:, 0])
        fy = interp1d(cumdist, path[:, 1])
        ftheta = interp1d(cumdist, path[:, 2])

        new_path = np.vstack((fx(new_s), fy(new_s), ftheta(new_s))).T
        return new_path
    
class ClothoidPath:
    def __init__(self, waypoints, lmbda=0.5, vehicle_half_width=1.0, vehicle_base_front=4.0):
        self.var = {
            "waypoints": waypoints,
            "lmbda": lmbda,
            "vehicle_half_width": vehicle_half_width,
            "vehicle_base_front": vehicle_base_front,
            "scenes": [],
            "paths": []
        }

    def path_planner(self):
        self.var["paths"].clear()
        self.var["scenes"].clear()
        wp = self.var["waypoints"]

        for i in range(len(wp) - 1):
            start = np.array(wp[i], dtype=np.float64)
            end = np.array(wp[i + 1], dtype=np.float64)

            try:
                # Try symmetric first
                scene_a, scene_b = solveDualScene(start, end,
                                                lmbda=self.var["lmbda"],
                                                vehicle_half_width=self.var["vehicle_half_width"],
                                                vehicle_base_front=self.var["vehicle_base_front"],
                                                same_lmbda=True)
                print(f"[INFO] Segment {i}: Symmetric solved.")
                full_path = np.vstack((scene_a.fullCurve(), scene_b.fullCurve()))
            except Exception as e1:
                try:
                    # Fallback: try unsymmetric
                    scene_a, scene_b = solveDualSceneUnsymmetric(start, end,
                                                                lmbda=self.var["lmbda"],
                                                                vehicle_half_width=self.var["vehicle_half_width"],
                                                                vehicle_base_front=self.var["vehicle_base_front"])
                    print(f"[INFO] Segment {i}: Unsymmetric solved.")
                    full_path = np.vstack((scene_a.fullCurve(), scene_b.fullCurve()))
                except Exception as e2:
                    try:
                        # Try only Scene A from start → end
                        scene_a = Scene(start, end, self.var["lmbda"], self.var["vehicle_half_width"], self.var["vehicle_base_front"])
                        full_path = scene_a.fullCurve()
                        scene_b = None
                        print(f"[INFO] Segment {i}: Only Scene A used.")
                    except Exception as a_e:
                        print(f"[WARNING] Segment {i} failed: {a_e}")
                        fallback = np.linspace(start[:2], end[:2], 50)
                        self.var["paths"].append(fallback)
                        self.var["scenes"].append((None, None))
                        continue

                    # # Final fallback: straight line
                    # print(f"[WARNING] Segment {i} failed (both symmetric and unsymmetric): {e2}")
                    # fallback = np.linspace(start[:2], end[:2], 50)
                    # self.var["paths"].append(fallback)
                    # self.var["scenes"].append((None, None))
                    # continue

            
            self.var["paths"].append(full_path)
            self.var["scenes"].append((scene_a, scene_b))



            # except Exception as dual_e:
                



    def plot(self):
        plt.figure()
        for path in self.var["paths"]:
            plt.plot(path[:, 0], path[:, 1], linewidth=2)

        for i, (x, y, _) in enumerate(self.var["waypoints"]):
            plt.plot(x, y, 'ko')
            plt.text(x + 0.1, y + 0.1, str(i), fontsize=10, color='red')

        plt.axis("equal")
        plt.title("Clothoid Path through Waypoints")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.show()


if __name__ == "__main__":



    # waypoints = [[132, 132], [146.472, 145.804], [153.885, 164.38 ], [173.486, 168.355], [174.36 , 188.336], [161.647, 203.776], [167.087, 223.022], [184.257, 233.278], [204.151, 231.219], [223.545, 226.335], [240.213, 237.388], [250.631, 254.461], [263.377, 269.873], [281.547, 278.231], [301.234, 281.755], [316.697, 294.439], [336.075, 299.388], [352.121, 311.326], [367.261, 324.394], [387.1 , 326.93], [405.61 , 334.505], [425.579, 333.398], [425.831, 313.399], [400, 300]]

    # waypoints=  [[132., 132.], [151.495, 136.465], [146.84 , 155.916], [133.95 , 171.208], [146.035, 187.143], [164.628, 194.512], [174.572, 211.865], [190.074, 224.501], [210.074, 224.373], [222.594, 239.969], [241.72 , 234.121], [259.085, 244.044], [277.442, 251.983], [296.869, 247.23 ], [316.534, 250.875], [329.851, 265.797], [338.146, 283.995], [357.625, 288.531], [358.899, 308.49 ], [377.574, 315.65 ], [393.159, 328.183], [400., 300.]]

    grid = np.load('image.npy')
    

    start = np.array([132.0, 132.0])
    goal = np.array([400.0, 300])

    numIterations = 2000
    stepSize = 20

    waypoints = RRT(grid, start, goal, numIterations, stepSize)

    def compute_heading_from_waypoints(waypoints):
        waypoints_with_theta = []
        for i in range(len(waypoints)):
            x, y = waypoints[i]
            if i < len(waypoints) - 1:
                dx = waypoints[i+1][0] - x
                dy = waypoints[i+1][1] - y
                theta = math.atan2(dy, dx)
            else:
                # use previous direction for last point
                dx = x - waypoints[i-1][0]
                dy = y - waypoints[i-1][1]
                theta = math.atan2(dy, dx)
            waypoints_with_theta.append([x, y, theta])
        return waypoints_with_theta



    waypoints_with_theta = compute_heading_from_waypoints(waypoints)
        
    
    planner = ClothoidPath(waypoints_with_theta, lmbda=0.1)
    planner.path_planner()
    # planner.plot()
    show_plot_nonblocking(planner)

    # print(planner.var["paths"])

    print("moving the robot as planned in waypoints")

    rclpy.init()
    motion_model = MotionModel()
    # commands = motion_model.compute_steering_commands(planner.var["paths"], speed=1.2, wheelbase=2.5)

    dense_paths = [motion_model.densify_path(p, spacing=0.05) for p in planner.var["paths"]]
    commands = motion_model.generate_time_aligned_commands(dense_paths)

    # Simulate in fixed time loop
    

    for v, steer in commands:
        motion_model.drive(v, steer)
        time.sleep(0.05)
    
    motion_model.drive(0.0, 0.0)