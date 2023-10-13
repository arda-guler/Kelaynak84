import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import math
from scipy.spatial.transform import Rotation

class Camera:
    def __init__(self, name, pos, orient, active, lock=None):
        self.name = name
        self.pos = pos
        self.orient = orient
        self.active = active
        self.lock = lock
        self.offset_amount = 0
        
    def get_name(self):
        return self.name

    def get_pos(self):
        return self.pos

    def set_pos(self, new_pos):
        req_trans = new_pos - self.pos
        glTranslate(req_trans[0], req_trans[1], req_trans[2])
        self.pos = new_pos

    def move(self, movement):
        if not self.lock:
            glTranslate((movement[0] * self.orient[0][0]) + (movement[1] * self.orient[1][0]) + (movement[2] * self.orient[2][0]),
                        (movement[0] * self.orient[0][1]) + (movement[1] * self.orient[1][1]) + (movement[2] * self.orient[2][1]),
                        (movement[0] * self.orient[0][2]) + (movement[1] * self.orient[1][2]) + (movement[2] * self.orient[2][2]))

            self.pos = np.array([self.pos[0] + (movement[0] * self.orient[0][0]) + (movement[1] * self.orient[1][0]) + (movement[2] * self.orient[2][0]),
                                 self.pos[1] + (movement[0] * self.orient[0][1]) + (movement[1] * self.orient[1][1]) + (movement[2] * self.orient[2][1]),
                                 self.pos[2] + (movement[0] * self.orient[0][2]) + (movement[1] * self.orient[1][2]) + (movement[2] * self.orient[2][2])])

        else:
            # this handles zooming in and out
            self.offset_amount += movement[2]
            if self.offset_amount <= 0:
                self.offset_amount -= movement[2]

    def get_orient(self):
        return self.orient
    
    def get_active(self):
        return self.active

    def activate(self):
        self.active = True

    def deactivate(self):
        self.active = False

    def get_lock(self):
        return self.lock

    def rotate(self, rotation):
        about_pos = self.pos
        
        glTranslate(-about_pos[0], -about_pos[1], -about_pos[2])
        glRotate(-rotation[0], self.orient[0][0], self.orient[0][1], self.orient[0][2])
        glTranslate(about_pos[0], about_pos[1], about_pos[2])

        glTranslate(-about_pos[0], -about_pos[1], -about_pos[2])
        glRotate(-rotation[1], self.orient[1][0], self.orient[1][1], self.orient[1][2])
        glTranslate(about_pos[0], about_pos[1], about_pos[2])

        glTranslate(-about_pos[0], -about_pos[1], -about_pos[2])
        glRotate(-rotation[2], self.orient[2][0], self.orient[2][1], self.orient[2][2])
        glTranslate(about_pos[0], about_pos[1], about_pos[2])

        # Update the orientation matrix
        if rotation[0] != 0:
            axis = np.array([1, 0, 0])
            angle_rad = -np.deg2rad(rotation[0])
            rot_mtx = np.eye(3) + np.sin(angle_rad) * np.cross(np.eye(3), axis) + \
                       (1 - np.cos(angle_rad)) * np.outer(axis, axis)
            self.orient = np.dot(rot_mtx, self.orient)
        if rotation[1] != 0:
            axis = np.array([0, 1, 0])
            angle_rad = -np.deg2rad(rotation[1])
            rot_mtx = np.eye(3) + np.sin(angle_rad) * np.cross(np.eye(3), axis) + \
                       (1 - np.cos(angle_rad)) * np.outer(axis, axis)
            self.orient = np.dot(rot_mtx, self.orient)
        if rotation[2] != 0:
            axis = np.array([0, 0, 1])
            angle_rad = -np.deg2rad(rotation[2])
            rot_mtx = np.eye(3) + np.sin(angle_rad) * np.cross(np.eye(3), axis) + \
                       (1 - np.cos(angle_rad)) * np.outer(axis, axis)
            self.orient = np.dot(rot_mtx, self.orient)

    def lock_to_target(self, target):
        self.lock = target
        self.set_pos(-self.lock.pos - self.orient[2] * self.offset_amount)

    def unlock(self):
        self.lock = None

    def move_with_lock(self, dt):
        if self.lock:
            self.set_pos(-self.lock.pos - self.orient[2] * self.offset_amount)
            #rot = np.array([self.lock.ang_vel[0], -self.lock.ang_vel[1], self.lock.ang_vel[2]])
            #self.rotate(np.rad2deg(rot * dt))

    def rotate_with_lock(self, dt):
        if self.lock:
            if np.linalg.norm(self.lock.ang_vel) > 0:

                about_pos = -self.lock.pos
                rotation = -self.lock.ang_vel * dt * 180 / 3.14159
        
                glTranslate(-about_pos[0], -about_pos[1], -about_pos[2])
                glRotate(-rotation[0], self.orient[0][0], self.orient[0][1], self.orient[0][2])
                glTranslate(about_pos[0], about_pos[1], about_pos[2])

                glTranslate(-about_pos[0], -about_pos[1], -about_pos[2])
                glRotate(-rotation[1], self.orient[1][0], self.orient[1][1], self.orient[1][2])
                glTranslate(about_pos[0], about_pos[1], about_pos[2])

                glTranslate(-about_pos[0], -about_pos[1], -about_pos[2])
                glRotate(-rotation[2], self.orient[2][0], self.orient[2][1], self.orient[2][2])
                glTranslate(about_pos[0], about_pos[1], about_pos[2])
        
                self.orient = self.lock.orient
