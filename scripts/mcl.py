#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import time
import cv2
import math
import random
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as tick
import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Pose, PoseArray, Quaternion, Vector3


class Goal:

    def __init__(self, location, range):
        self.location = np.array(location)
        self.range = range

    def draw(self):
        x = self.location[0]
        y = self.location[1]
        plt.scatter(x, y, s=200, marker="x", label="goal",
                    color="black", alpha=0.3)


class GridMap():

    def __init__(self, map_image, resolution):
        # マップ画像
        self.map_image = map_image.T[:, ::-1]
        # 解像度 (マップ画像の1[px]の大きさ[m])
        self.resolution = resolution
        # マップ画像のサイズ[px]
        self.width = map_image.shape[1]
        self.height = map_image.shape[0]

        self.x = np.linspace(0, resolution * self.width, self.width + 1)
        self.y = np.linspace(0, resolution * self.height, self.height + 1)

        # 障害物があるセルの中心座標リスト
        self.obstacle_idxs = np.array(np.where(self.map_image > 90)).T
        self.obstacle_coordinate =\
            self.obstacle_idxs * self.resolution + self.resolution/2
    
    def draw(self):
        plt.pcolor(self.x, self.y, self.map_image.T, cmap=plt.cm.gray,
                   vmin=0, vmax=self.map_image.max())


class Particle:

    def __init__(self, pose, weight):
        self.pose = np.array(pose)
        self.weight = weight


class MCL():

    def __init__(self, particle_num, goal,
                 initial_pose=None, std_dev=0.05):
        self.estimate_pose = np.array([0.0, 0.0, 0.0])
        self.goal = goal
        self.particles = []
        if initial_pose is None:
            self.set_initial_pose(particle_num)
        else:
            for i in range(particle_num):
                self.particles.append(Particle(initial_pose,
                                               1.0/particle_num))
        self.std_dev = std_dev

    def state_transition(self, pose, u, dt):
        t0 = pose[2]
        nu, omega = u
        ns = np.random.normal(0, 0.02, 4)
        pnu = nu + ns[0]*math.sqrt(abs(nu)/dt)\
            + ns[1]*math.sqrt(abs(omega)/dt)
        pomega = omega + ns[2]*math.sqrt(abs(nu)/dt)\
            + ns[3]*math.sqrt(abs(omega)/dt)
        
        if math.fabs(omega) < 1e-10:
            return pose + np.array([pnu*math.cos(t0),
                                    pnu*math.sin(t0),
                                    pomega]) * dt
        else:
            return pose\
                + np.array([pnu/pomega*(math.sin(t0 + pomega*dt) - math.sin(t0)),
                            pnu/pomega*(-math.cos(t0 + pomega*dt) + math.cos(t0)),
                            pomega*dt])

    def motion_update(self, u, dt):
        for p in self.particles:
            p.pose = self.state_transition(p.pose, u, dt)
    
    def sensor_updata(self, scans, grid_map):
        for p in self.particles:
            p.weight = self.calc_likelihood(scans, grid_map, p.pose)
        self.resampling()

    def calc_likelihood(self, scans, grid_map, pose):
        if np.linalg.norm(self.goal.location - pose[:2]) < self.goal.range:
            return 1e-100
        else:
            return 1

        q = 1
        for k in scans:
            pos = pose[:2] + k[1] * np.array([np.cos(pose[2]+k[0]),
                                              np.sin(pose[2]+k[0])])
            dist = np.min([np.linalg.norm(pos-px)
                            for px in grid_map.obstacle_coordinate])
            q *= np.exp(-dist**2 / (2 * self.std_dev**2))\
                / (np.sqrt(2 * np.pi) * self.std_dev)
            plt.scatter(pos[0], pos[1],
                        marker="s", s=30, c="orange", alpha=0.3,
                        linewidths="0", edgecolors="red")
        
        return q
    
    def resampling(self):
        particle_num = len(self.particles)
        weights = [p.weight for p in self.particles]
        weight_sum = np.sum(weights)
        accum = np.cumsum(weights)
        if weight_sum < 1e-100:
            weights = [w + 1e-100 for w in weights]

        pointers = np.arange(np.random.rand() * weight_sum/particle_num,
                             weight_sum, weight_sum/particle_num)
        particles = []
        for ptr in pointers:
            idx = np.argmin(accum < ptr)
            particles.append(Particle(self.particles[idx].pose,
                                      1.0/particle_num))
        
        self.particles = particles

    def set_initial_pose(self, particle_num):
        initial_location = np.random.rand(particle_num, 2) * 2
        initial_rotation =\
            np.random.rand(particle_num, 1)*2*np.pi - np.pi
        initial_pose = np.hstack([initial_location, initial_rotation])
        for i in range(particle_num):
            self.particles.append(Particle(initial_pose[i],
                                            1.0/particle_num))
    
    def draw_particles(self):
        x = [p.pose[0] for p in self.particles]
        y = [p.pose[1] for p in self.particles]
        vx = [np.cos(p.pose[2]) for p in self.particles]
        vy = [np.sin(p.pose[2]) for p in self.particles]
        plt.quiver(x, y, vx, vy, color='red', label='particles', alpha=0.4)


class MCLROS():
    
    def __init__(self):
        Hz = 10.0
        i = 0
        rate = rospy.Rate(Hz)
        self.u = (0.0, 0.0)
        self.scans = np.array([[]])
        # マップ
        self.map_image = np.array([[0, 0, 0]])
        self.resolution = 0.01
        self.grid_map = GridMap(self.map_image, self.resolution)

        goal = Goal([1.0, 1.5], 0.1)

        # Subscriber
        self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist,
                                            self.cmd_vel_cb, queue_size=10)
        self.sub_scan = rospy.Subscriber("/scan", LaserScan,
                                         self.scan_cb, queue_size=1)
        self.sub_map = rospy.Subscriber("/map", OccupancyGrid,
                                         self.map_cb, queue_size=1)
        # Publisher
        self.pub_pose_array = rospy.Publisher("/particlecloud", PoseArray,
                                              latch=True, queue_size=1)

        # 自己位置推定
        self.mcl = MCL(1, goal, initial_pose=[1.0, 0.5, 0.0])

        while not rospy.is_shutdown():
            if i > 10:
                pose_array = self.create_pose_array(self.mcl.particles)
                self.pub_pose_array.publish(pose_array)
                self.mcl.sensor_updata(self.scans, self.grid_map)
                i = 0
            i += 1
            rate.sleep()

    def cmd_vel_cb(self, cmd_vel):
        self.u = (cmd_vel.linear.x, cmd_vel.angular.z)
        self.mcl.motion_update(self.u, 0.2) 
    
    def scan_cb(self, scan):
        ranges = np.array(scan.ranges)
        angles = np.arange(scan.angle_min, scan.angle_max+scan.angle_increment,
                           scan.angle_increment)
        scans_with_inf = (np.vstack((angles, ranges)).T)[60:359:240]
        self.scans = scans_with_inf[~np.isinf(scans_with_inf).any(axis=1)]

    def map_cb(self, grid_map):
        self.map_image = np.array(grid_map.data).reshape(grid_map.info.height,
                                                         grid_map.info.width)
        self.resolution = grid_map.info.resolution
        self.grid_map = GridMap(self.map_image, self.resolution)
    
    def create_pose_array(self, particles):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "/map"
        for p in particles:
            pose = Pose()
            pose.position.x = p.pose[0]
            pose.position.y = p.pose[1]
            pose.orientation =\
                self.euler2quaternion(Vector3(0.0, 0.0, p.pose[2]))
            pose_array.poses.append(pose)

        return pose_array

    def euler2quaternion(self, euler):
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)

        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    

if __name__ == '__main__':
    rospy.init_node("mcl")
    try:
        mcl_ros = MCLROS()
    except rospy.ROSInterruptException:
        pass
