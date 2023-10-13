import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import numpy as np

from ui import *

class Color:
    def __init__(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b

def drawOrigin():
    glBegin(GL_LINES)
    glColor(1,0,0)
    glVertex3f(0,0,0)
    glVertex3f(100,0,0)
    glColor(0,1,0)
    glVertex3f(0,0,0)
    glVertex3f(0,100,0)
    glColor(0,0,1)
    glVertex3f(0,0,0)
    glVertex3f(0,0,100)
    glEnd()

def drawControls(cam, ctrl_state, first_person_ui):
    glPointSize(3)
    
    # aileron, elevator
    drawRectangle2D(7, -3, 10, -6, Color(1, 0, 1), cam, first_person_ui)
    drawPoint2D(8.5 - ctrl_state[0], -4.5 - ctrl_state[1], Color(1, 0, 1), cam, first_person_ui)
    # rudder
    drawRectangle2D(7, -2, 10, -3, Color(1, 0, 1), cam, first_person_ui)
    drawPoint2D(8.5 + ctrl_state[2], -2.5, Color(1, 0, 1), cam, first_person_ui)
    
    glPointSize(1)

def drawPoint(p, color):

    glColor(color.r, color.g, color.b)
        
    glPushMatrix()
    glTranslatef(p.pos[0], p.pos[1], p.pos[2])

    glBegin(GL_POINTS)
    glVertex3f(0, 0, 0)
    glEnd()

    glPopMatrix()

def drawFlatland(cam, flatland, size=2000, divisions=50):
    spacing = 10**(int(math.log(abs(cam.pos[1] - flatland.height) + 2, 10)) + 1)
    # size = abs(cam.pos.y) * 10

    # subdivisions
    scene_spacing = spacing / 10
    N = divisions * 10
    corner_x = (-cam.pos[0] - N * 0.5 * scene_spacing) + cam.pos[0] % (scene_spacing)
    corner_z = (-cam.pos[2] - N * 0.5 * scene_spacing) + cam.pos[2] % (scene_spacing)

    glColor(flatland.color.r/2, flatland.color.g/2, flatland.color.b/2)
    glBegin(GL_LINES)
    for i in range(N + 1):
        cx = corner_x + i * scene_spacing
        z0 = corner_z
        z1 = corner_z + N * scene_spacing
        glVertex3f(cx, flatland.height, z0)
        glVertex3f(cx, flatland.height, z1)

    for i in range(N + 1):
        x0 = corner_x
        x1 = corner_x + N * scene_spacing
        cz = corner_z + i * scene_spacing
        glVertex3f(x0, flatland.height, cz)
        glVertex3f(x1, flatland.height, cz)
    glEnd()

    # superdivisions
    scene_spacing = spacing
    N = divisions
    corner_x = (-cam.pos[0] - N * 0.5 * scene_spacing) + cam.pos[0] % (scene_spacing)
    corner_z = (-cam.pos[2] - N * 0.5 * scene_spacing) + cam.pos[2] % (scene_spacing)

    glColor(flatland.color.r, flatland.color.g, flatland.color.b)
    glBegin(GL_LINES)
    for i in range(N + 1):
        cx = corner_x + i * scene_spacing
        z0 = corner_z
        z1 = corner_z + N * scene_spacing
        glVertex3f(cx, flatland.height, z0)
        glVertex3f(cx, flatland.height, z1)

    for i in range(N + 1):
        x0 = corner_x
        x1 = corner_x + N * scene_spacing
        cz = corner_z + i * scene_spacing
        glVertex3f(x0, flatland.height, cz)
        glVertex3f(x1, flatland.height, cz)
    glEnd()

def drawForces(forces):
    
    for f in forces:
        glPushMatrix()

        scaler = 0.2
        start_position = f.point.pos
        end_position = f.point.pos + f.force
        f_vector = f.force * scaler
        
        f_dir = f_vector.normalized()
        arrowhead_start = f.force * scaler * 0.8

        if not f_dir.cross(vec3(1,0,0)) == vec3(0,0,0):
            arrowhead_vector1 = f_dir.cross(vec3(1,0,0))
        else:
            arrowhead_vector1 = f_dir.cross(vec3(0,1,0))

        arrowhead_vector2 = arrowhead_vector1.cross(f_dir)

        arrowhead_vector1 = arrowhead_vector1 * f.force.mag() * scaler * 0.1
        arrowhead_vector2 = arrowhead_vector2 * f.force.mag() * scaler * 0.1
            
        arrowhead_pt1 = arrowhead_start + arrowhead_vector1
        arrowhead_pt2 = arrowhead_start - arrowhead_vector1

        arrowhead_pt3 = arrowhead_start + arrowhead_vector2
        arrowhead_pt4 = arrowhead_start - arrowhead_vector2
        
        glTranslate(start_position.x, start_position.y, start_position.z)
        glColor(1,0,1)

        glBegin(GL_LINES)

        glVertex3f(0,0,0)
        glVertex3f(f_vector.x, f_vector.y, f_vector.z)

        glVertex3f(arrowhead_pt1.x, arrowhead_pt1.y, arrowhead_pt1.z)
        glVertex3f(arrowhead_pt3.x, arrowhead_pt3.y, arrowhead_pt3.z)

        glVertex3f(arrowhead_pt2.x, arrowhead_pt2.y, arrowhead_pt2.z)
        glVertex3f(arrowhead_pt4.x, arrowhead_pt4.y, arrowhead_pt4.z)

        glVertex3f(arrowhead_pt2.x, arrowhead_pt2.y, arrowhead_pt2.z)
        glVertex3f(arrowhead_pt3.x, arrowhead_pt3.y, arrowhead_pt3.z)

        glVertex3f(arrowhead_pt1.x, arrowhead_pt1.y, arrowhead_pt1.z)
        glVertex3f(arrowhead_pt4.x, arrowhead_pt4.y, arrowhead_pt4.z)

        glVertex3f(arrowhead_pt1.x, arrowhead_pt1.y, arrowhead_pt1.z)
        glVertex3f(f_vector.x, f_vector.y, f_vector.z)

        glVertex3f(arrowhead_pt2.x, arrowhead_pt2.y, arrowhead_pt2.z)
        glVertex3f(f_vector.x, f_vector.y, f_vector.z)

        glVertex3f(arrowhead_pt3.x, arrowhead_pt3.y, arrowhead_pt3.z)
        glVertex3f(f_vector.x, f_vector.y, f_vector.z)

        glVertex3f(arrowhead_pt4.x, arrowhead_pt4.y, arrowhead_pt4.z)
        glVertex3f(f_vector.x, f_vector.y, f_vector.z)

        glEnd()

        glPopMatrix()

def drawModel(model, pos, orient, scale, color=Color(1, 1, 1)):
    glPushMatrix()
    glTranslatef(pos[0], pos[1], pos[2])
    glColor(color.r, color.g, color.b)

    glBegin(GL_LINES)
    for lines in model.lines:
        v1 = model.vertices[lines[0]]
        v2 = model.vertices[lines[1]]

        v1_rot = np.array([[v1[0] * orient[0][0] + v1[1] * orient[1][0] + v1[2] * orient[2][0]],
                           [v1[0] * orient[0][1] + v1[1] * orient[1][1] + v1[2] * orient[2][1]],
                           [v1[0] * orient[0][2] + v1[1] * orient[1][2] + v1[2] * orient[2][2]]])

        v2_rot = np.array([[v2[0] * orient[0][0] + v2[1] * orient[1][0] + v2[2] * orient[2][0]],
                           [v2[0] * orient[0][1] + v2[1] * orient[1][1] + v2[2] * orient[2][1]],
                           [v2[0] * orient[0][2] + v2[1] * orient[1][2] + v2[2] * orient[2][2]]])
        
        glVertex3f(v1_rot[0], v1_rot[1], v1_rot[2])
        glVertex3f(v2_rot[0], v2_rot[1], v2_rot[2])
    glEnd()

    
    for faces in model.faces:
        glBegin(GL_POLYGON)
        vnum = len(faces)
        for i in range(vnum):
            v = model.vertices[faces[i]]
            v_rot = np.array([[v[0] * orient[0][0] + v[1] * orient[1][0] + v[2] * orient[2][0]],
                              [v[0] * orient[0][1] + v[1] * orient[1][1] + v[2] * orient[2][1]],
                              [v[0] * orient[0][2] + v[1] * orient[1][2] + v[2] * orient[2][2]]])

            glVertex3f(v_rot[0], v_rot[1], v_rot[2])
        glEnd()
    
    glPopMatrix()

def drawScene(cam, flatland, bodies, cities, scenery_objects, ctrl_state, first_person_ui=False):
    drawFlatland(cam, flatland)
    for c in cities:
        city_dist = np.linalg.norm(c.pos - cam.pos)
        if city_dist < c.size * 1e4:
            for b in c.buildings:
                drawModel(b.model, b.pos, np.eye(3), 1, b.color)
        elif city_dist < c.size * 1e6:
            for b in c.buildings:
                drawPoint(b, b.color)
    
    for o in scenery_objects:
        drawModel(o.model, o.pos, np.eye(3), 1, o.color)

    for b in bodies:
        if np.linalg.norm(b.pos - cam.pos) < 1e5:
            drawModel(b.model, b.pos, b.orient, 1, Color(1, 0, 0))
        else:
            drawPoint(o, Color(1, 0, 0))
    drawControls(cam, ctrl_state, first_person_ui)

