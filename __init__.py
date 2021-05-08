#!/usr/bin/env python3
bl_info = {
    "name": "Mario Tester",
    "description": "Allows you to test Mario 64 levels.",
    "author": "Wiseguy",
    "version": (1, 0),
    "blender": (2, 92, 0),
    "location": "View3D",
    "category": "Object",
}

from .surface_types import *
from .inputs import *
import time
import threading
import sys
import os
import math
import itertools
from ctypes import *

import bpy
import bmesh
import mathutils

pluginPath = os.path.dirname(os.path.realpath(__file__))

library = 'libmario.so'
if sys.platform == 'win32': library = 'libmario.dll'
if sys.platform == 'darwin': library = 'libmario.dynlib'

libmario = CDLL(os.path.join(pluginPath, library))

Vec3f = (c_float * 3)
Vec3s = (c_short * 3)
class AnimData(Structure):
    _fields_ = [
        ('root_translation', Vec3f),
        ('bone_rotations', Vec3f * 20)
    ]

class Surface(Structure):
    _fields_ = [
        ('type', c_int16),
        ('force', c_int16),
        ('flags', c_int8),
        ('room', c_int8),
        ('lowerY', c_int16),
        ('upperY', c_int16),
        ('vertex1', Vec3s),
        ('vertex2', Vec3s),
        ('vertex3', Vec3s),
        ('normal', Vec3f),
        ('origin_offset', c_float),
        ('object', c_void_p)
    ]
    
FindFloorHandlerType = CFUNCTYPE(c_float, c_float, c_float, c_float, POINTER(Surface), POINTER(c_int32))
FindCeilHandlerType = CFUNCTYPE(c_float, c_float, c_float, c_float, POINTER(Surface), POINTER(c_int32))
FindWallsHandlerType = CFUNCTYPE(c_int32, c_float, c_float, c_float, c_float, c_float, POINTER(Surface), POINTER(c_float))

libmario.init.restype = None
libmario.init.artypes = [FindFloorHandlerType, FindCeilHandlerType, FindWallsHandlerType]

libmario.step.restype = None
libmario.step.artypes = [c_int32, c_float, c_float]

libmario.getMarioPosition.restype = None
libmario.getMarioPosition.artypes = [Vec3f]

libmario.getMarioVelocity.restype = None
libmario.getMarioVelocity.artypes = [Vec3f]

libmario.getMarioRotation.restype = None
libmario.getMarioRotation.artypes = [Vec3f]

libmario.getMarioScale.restype = None
libmario.getMarioScale.artypes = [Vec3f]

libmario.getMarioTorsoRotation.restype = None
libmario.getMarioTorsoRotation.artypes = [Vec3f]

libmario.getMarioAnimFrame.restype = c_int32
libmario.getMarioAnimFrame.artypes = []

libmario.getMarioAnimIndex.restype = c_int32
libmario.getMarioAnimIndex.artypes = []

libmario.getMarioAnimData.restype = None
libmario.getMarioAnimData.artypes = [POINTER(AnimData)]

CONT_A      = 0x8000
CONT_B      = 0x4000
CONT_G      = 0x2000
CONT_START  = 0x1000
CONT_UP     = 0x0800
CONT_DOWN   = 0x0400
CONT_LEFT   = 0x0200
CONT_RIGHT  = 0x0100
CONT_L      = 0x0020
CONT_R      = 0x0010
CONT_E      = 0x0008
CONT_D      = 0x0004
CONT_C      = 0x0002
CONT_F      = 0x0001

A_BUTTON     = CONT_A
B_BUTTON     = CONT_B
L_TRIG       = CONT_L
R_TRIG       = CONT_R
Z_TRIG       = CONT_G
START_BUTTON = CONT_START
U_JPAD       = CONT_UP
L_JPAD       = CONT_LEFT
R_JPAD       = CONT_RIGHT
D_JPAD       = CONT_DOWN
U_CBUTTONS   = CONT_E
L_CBUTTONS   = CONT_C
R_CBUTTONS   = CONT_F
D_CBUTTONS   = CONT_D

SURFACE_FLAG_DYNAMIC          = (1 << 0)
SURFACE_FLAG_NO_CAM_COLLISION = (1 << 1)
SURFACE_FLAG_X_PROJECTION     = (1 << 3)

events = []
_t = None

bone_names = [
    'root',
    '000-offset',
    '000-offset.001',
    '000-offset.002',
    '002-offset',
    '000-offset.003',
    '000-offset.004',
    '000-offset.005',
    '004-offset',
    '000-offset.006',
    '000-offset.007',
    '000-offset.008',
    '004-offset.001',
    '000-offset.009',
    '000-offset.010',
    '000-offset.011',
    '006-offset',
    '000-offset.012',
    '000-offset.013',
    '000-offset.014',
]

def worker():
    global events
    while True:
        events.append(get_gamepad())

def get_surface_type(name):
    return surface_type_dict.get(name, 0x0000)

class MarioTester(bpy.types.Operator):
    """Mario Tester"""
    bl_idname = "view3d.mario_tester"
    bl_label = "Mario Tester"

    _timer = None
    mario_obj = None

    stick_x = 0.0
    stick_y = 0.0
    buttons = 0

    start_time = 0
    last_frame = 0

    collision_data = []
    # TODO split surfaces into walls/floors/ceilings
    floor_bvh_trees = dict()

    depsgraph = None

    # Creatures a closure that allows the function to access the MarioTester instance
    def create_find_floor_handler(self):

        def find_floor(x, y, z, surface_out, found_out):
            ray_origin = mathutils.Vector((x / 100.0, -z / 100.0, y / 100.0 + 0.78))
            ray_dir = mathutils.Vector((0, 0, -1))
            hit_mats = None
            hit_bvh = None
            hit_dist = sys.float_info.max
            hit_height = -11000.0 / 100.0
            hit_norm = mathutils.Vector((0.0, 0.0, 1.0))
            hit_surface = 0x0000

            for bvh, (material_indices, mats) in self.floor_bvh_trees.items():
                cur_pos, cur_norm, index, cur_dist = bvh.ray_cast(ray_origin, ray_dir)
                if cur_pos and cur_dist < hit_dist:
                    hit_dist = cur_dist
                    hit_bvh = bvh
                    hit_mats = mats
                    hit_height = cur_pos.z
                    hit_norm = cur_norm
                    hit_surface = get_surface_type(mats[material_indices[index]])

            if hit_bvh:
                found_out[0] = 1
                surface_out[0].vertex1[0] = -100
                surface_out[0].vertex1[1] = 0
                surface_out[0].vertex1[2] = -100

                surface_out[0].vertex2[0] = 100
                surface_out[0].vertex2[1] = 0
                surface_out[0].vertex2[2] = -100

                surface_out[0].vertex3[0] = 100
                surface_out[0].vertex3[1] = 0
                surface_out[0].vertex3[2] = 100

                surface_out[0].normal[0] = hit_norm[0]
                surface_out[0].normal[1] = hit_norm[2]
                surface_out[0].normal[2] = -hit_norm[1]

                surface_out[0].origin_offset = hit_height * 100

                surface_out[0].type = hit_surface
                surface_out[0].flags = SURFACE_FLAG_DYNAMIC

            else:
                found_out[0] = 0
            
            return hit_height * 100

        # Return closure
        return find_floor
        
    # Creatures a closure that allows the function to access the MarioTester instance
    def create_find_ceil_handler(self):

        def find_ceil(x, y, z, surface_out, found_out):
            ray_origin = mathutils.Vector((x / 100.0, -z / 100.0, y / 100.0 - 0.78))
            ray_dir = mathutils.Vector((0, 0, 1))
            hit_mats = None
            hit_bvh = None
            hit_dist = sys.float_info.max
            hit_height = 20000.0 / 100.0
            hit_norm = mathutils.Vector((0.0, 0.0, -1.0))
            hit_surface = 0x0000

            for bvh, (material_indices, mats) in self.ceil_bvh_trees.items():
                cur_pos, cur_norm, index, cur_dist = bvh.ray_cast(ray_origin, ray_dir)
                if cur_pos and cur_dist < hit_dist:
                    hit_dist = cur_dist
                    hit_bvh = bvh
                    hit_mats = mats
                    hit_height = cur_pos.z
                    hit_norm = cur_norm
                    hit_surface = get_surface_type(mats[material_indices[index]])

            if hit_bvh:
                found_out[0] = 1
                surface_out[0].vertex1[0] = -100
                surface_out[0].vertex1[1] = 0
                surface_out[0].vertex1[2] = -100

                surface_out[0].vertex2[0] = 100
                surface_out[0].vertex2[1] = 0
                surface_out[0].vertex2[2] = -100

                surface_out[0].vertex3[0] = 100
                surface_out[0].vertex3[1] = 0
                surface_out[0].vertex3[2] = 100

                surface_out[0].normal[0] = hit_norm[0]
                surface_out[0].normal[1] = hit_norm[2]
                surface_out[0].normal[2] = -hit_norm[1]

                surface_out[0].origin_offset = hit_height * 100

                surface_out[0].type = hit_surface
                surface_out[0].flags = SURFACE_FLAG_DYNAMIC

            else:
                found_out[0] = 0
            
            return hit_height * 100

        # Return closure
        return find_ceil
        
    # Creatures a closure that allows the function to access the MarioTester instance
    def create_find_wall_collisions_handler(self):

        def find_wall_collisions(x, y, z, offsetY, radius, surfaces_out, pos_out):
            if radius > 200.0: radius = 200.0
            real_radius = radius / 100.0

            pos_x_ray_dir = mathutils.Vector(( 1,  0, 0))
            pos_x_ray_origin = mathutils.Vector((x / 100.0 - real_radius, -z / 100.0, (y + offsetY) / 100.0))

            neg_x_ray_dir = mathutils.Vector((-1,  0, 0))
            neg_x_ray_origin = mathutils.Vector((x / 100.0 + real_radius, -z / 100.0, (y + offsetY) / 100.0))
            
            pos_z_ray_dir = mathutils.Vector(( 0, -1, 0)) # Named based on sm64 coordinate system
            pos_z_ray_origin = mathutils.Vector((x / 100.0, -z / 100.0 + real_radius, (y + offsetY) / 100.0))
            
            neg_z_ray_dir = mathutils.Vector(( 0,  1, 0))
            neg_z_ray_origin = mathutils.Vector((x / 100.0, -z / 100.0 - real_radius, (y + offsetY) / 100.0))

            pos = mathutils.Vector((x, y, z))

            num_hits = 0

            # print('ray start: ' + str(pos_x_ray_origin))
            # print('ray dir: ' + str(pos_x_ray_dir))

            # raycast positive ray direction against negative normal direction
            for bvh, (material_indices, mats) in self.wall_neg_x_bvh_trees.items():
                cur_pos, cur_norm, index, cur_dist = bvh.ray_cast(pos_x_ray_origin, pos_x_ray_dir, real_radius * 2)
                if cur_pos:
                    if num_hits < 4:
                        surfaces_out[num_hits].normal[0] = cur_norm[0]
                        surfaces_out[num_hits].normal[1] = cur_norm[2]
                        surfaces_out[num_hits].normal[2] = -cur_norm[1]
                        pos += mathutils.Vector((cur_dist * 100.0 - (radius * 2), 0, 0))
                        surfaces_out[num_hits].type = get_surface_type(mats[material_indices[index]])
                        surfaces_out[num_hits].flags = SURFACE_FLAG_DYNAMIC
                    num_hits += 1
            
            # raycast negative ray direction against positive normal direction
            for bvh, (material_indices, mats) in self.wall_pos_x_bvh_trees.items():
                cur_pos, cur_norm, index, cur_dist = bvh.ray_cast(neg_x_ray_origin, neg_x_ray_dir, real_radius * 2)
                if cur_pos:
                    if num_hits < 4:
                        surfaces_out[num_hits].normal[0] = cur_norm[0]
                        surfaces_out[num_hits].normal[1] = cur_norm[2]
                        surfaces_out[num_hits].normal[2] = -cur_norm[1]
                        pos -= mathutils.Vector((cur_dist * 100.0 - (radius * 2), 0, 0))
                        surfaces_out[num_hits].type = get_surface_type(mats[material_indices[index]])
                        surfaces_out[num_hits].flags = SURFACE_FLAG_DYNAMIC
                    num_hits += 1
                  
            # raycast positive ray direction against negative normal direction  
            for bvh, (material_indices, mats) in self.wall_neg_z_bvh_trees.items():
                cur_pos, cur_norm, index, cur_dist = bvh.ray_cast(pos_z_ray_origin, pos_z_ray_dir, real_radius * 2)
                if cur_pos:
                    if num_hits < 4:
                        surfaces_out[num_hits].normal[0] = cur_norm[0]
                        surfaces_out[num_hits].normal[1] = cur_norm[2]
                        surfaces_out[num_hits].normal[2] = -cur_norm[1]
                        pos += mathutils.Vector((0, 0, cur_dist * 100.0 - (radius * 2)))
                        surfaces_out[num_hits].type = get_surface_type(mats[material_indices[index]])
                        surfaces_out[num_hits].flags = SURFACE_FLAG_DYNAMIC
                    num_hits += 1
            
            # raycast negative ray direction against positive normal direction
            for bvh, (material_indices, mats) in self.wall_pos_z_bvh_trees.items():
                cur_pos, cur_norm, index, cur_dist = bvh.ray_cast(neg_z_ray_origin, neg_z_ray_dir, real_radius * 2)
                if cur_pos:
                    if num_hits < 4:
                        surfaces_out[num_hits].normal[0] = cur_norm[0]
                        surfaces_out[num_hits].normal[1] = cur_norm[2]
                        surfaces_out[num_hits].normal[2] = -cur_norm[1]
                        pos -= mathutils.Vector((0, 0, cur_dist * 100.0 - (radius * 2)))
                        surfaces_out[num_hits].type = get_surface_type(mats[material_indices[index]])
                        surfaces_out[num_hits].flags = SURFACE_FLAG_DYNAMIC
                    num_hits += 1
            
            # print('num_hits: ' + str(num_hits))

            pos_out[0] = pos[0]
            pos_out[1] = pos[1]
            pos_out[2] = pos[2]
            
            return num_hits

        # Return closure
        return find_wall_collisions

    def process_updates(self, context):
        scene_obj_set = set(context.scene.objects.values())

        collision_objs = [obj for obj in scene_obj_set if (obj.type == 'MESH' and obj.visible_get() and not obj.ignore_collision)]

        self.floor_bvh_trees = dict()
        self.ceil_bvh_trees = dict()
        self.wall_pos_x_bvh_trees = dict()
        self.wall_neg_x_bvh_trees = dict()
        self.wall_pos_z_bvh_trees = dict()
        self.wall_neg_z_bvh_trees = dict()

        for obj in collision_objs:
            mesh = obj.data
            if mesh.name_full == 'skinned.001': # todo not this
                continue
            # print(mesh.name_full)
            materials = list(mesh.materials)
            if not any(mat.is_f3d for mat in materials): # mesh has no f3d materials
                continue
            cur_bmesh = bmesh.new()
            cur_bmesh.from_mesh(mesh)
            non_f3d_faces = [face for face in cur_bmesh.faces if not materials[face.material_index].is_f3d]
            bmesh.ops.delete(cur_bmesh, geom=non_f3d_faces, context='FACES_ONLY')
            cur_bmesh.transform(obj.matrix_world)
            cur_bmesh.normal_update()

            surface_types = [mat.collision_type_simple for mat in materials]
            
            cur_floor_bmesh = cur_bmesh.copy()
            cur_ceil_bmesh = cur_bmesh.copy()
            cur_walls_pos_x_bmesh = cur_bmesh.copy()
            cur_walls_neg_x_bmesh = cur_bmesh.copy()
            cur_walls_pos_z_bmesh = cur_bmesh.copy() # z in sm64 coordinate space, -y in blender coordinate space
            cur_walls_neg_z_bmesh = cur_bmesh.copy()

            not_floor_faces = [face for face in cur_floor_bmesh.faces if not (face.normal.z > 0.01)]
            not_ceil_faces = [face for face in cur_ceil_bmesh.faces if not (face.normal.z < -0.01)]
            not_pos_x_walls_faces = [face for face in cur_walls_pos_x_bmesh.faces if (face.normal.z < -0.01 or face.normal.z > 0.01 or face.normal.x <=  0.707)]
            not_neg_x_walls_faces = [face for face in cur_walls_neg_x_bmesh.faces if (face.normal.z < -0.01 or face.normal.z > 0.01 or face.normal.x >= -0.707)]
            # [face for face in cur_walls_x_bmesh.faces if (face.normal.z < -0.01 or face.normal.z > 0.01 or not (face.normal.x >= -0.707 and face.normal.x <= 0.707))]
            not_pos_z_walls_faces = [face for face in cur_walls_pos_z_bmesh.faces if (face.normal.z < -0.01 or face.normal.z > 0.01 or face.normal.x < -0.707 or face.normal.x > 0.707 or face.normal.y >= 0)]
            not_neg_z_walls_faces = [face for face in cur_walls_neg_z_bmesh.faces if (face.normal.z < -0.01 or face.normal.z > 0.01 or face.normal.x < -0.707 or face.normal.x > 0.707 or face.normal.y  < 0)]

            # delete floors/ceils/walls from corresponding bmeshes
            bmesh.ops.delete(cur_floor_bmesh, geom=not_floor_faces, context='FACES_ONLY')
            bmesh.ops.delete(cur_ceil_bmesh, geom=not_ceil_faces, context='FACES_ONLY')
            bmesh.ops.delete(cur_walls_pos_x_bmesh, geom=not_pos_x_walls_faces, context='FACES_ONLY')
            bmesh.ops.delete(cur_walls_neg_x_bmesh, geom=not_neg_x_walls_faces, context='FACES_ONLY')
            bmesh.ops.delete(cur_walls_pos_z_bmesh, geom=not_pos_z_walls_faces, context='FACES_ONLY')
            bmesh.ops.delete(cur_walls_neg_z_bmesh, geom=not_neg_z_walls_faces, context='FACES_ONLY')

            cur_bmesh.free()
            cur_walls_pos_x_bmesh.normal_update()
            cur_walls_neg_x_bmesh.normal_update()
            self.floor_bvh_trees[mathutils.bvhtree.BVHTree.FromBMesh(cur_floor_bmesh)] = ([face.material_index for face in cur_floor_bmesh.faces], surface_types)
            self.ceil_bvh_trees[mathutils.bvhtree.BVHTree.FromBMesh(cur_ceil_bmesh)] = ([face.material_index for face in cur_ceil_bmesh.faces], surface_types)
            self.wall_pos_x_bvh_trees[mathutils.bvhtree.BVHTree.FromBMesh(cur_walls_pos_x_bmesh)] = ([face.material_index for face in cur_walls_pos_x_bmesh.faces], surface_types)
            self.wall_neg_x_bvh_trees[mathutils.bvhtree.BVHTree.FromBMesh(cur_walls_neg_x_bmesh)] = ([face.material_index for face in cur_walls_neg_x_bmesh.faces], surface_types)
            self.wall_pos_z_bvh_trees[mathutils.bvhtree.BVHTree.FromBMesh(cur_walls_pos_z_bmesh)] = ([face.material_index for face in cur_walls_pos_z_bmesh.faces], surface_types)
            self.wall_neg_z_bvh_trees[mathutils.bvhtree.BVHTree.FromBMesh(cur_walls_neg_z_bmesh)] = ([face.material_index for face in cur_walls_neg_z_bmesh.faces], surface_types)

            # print('pos x faces: ' + str(len(cur_walls_pos_x_bmesh.faces)))
            # print('neg x faces: ' + str(len(cur_walls_neg_x_bmesh.faces)))
            # print('pos z faces: ' + str(len(cur_walls_pos_z_bmesh.faces)))
            # print('neg z faces: ' + str(len(cur_walls_neg_z_bmesh.faces)))
            
            cur_floor_bmesh.free()
            cur_ceil_bmesh.free()
            cur_walls_pos_x_bmesh.free()
            cur_walls_neg_x_bmesh.free()
            cur_walls_pos_z_bmesh.free()
            cur_walls_neg_z_bmesh.free()

    def execute(self, context):
        global _t
        global events

        if not hasattr(bpy.types.Object, 'ignore_collision'):
            raise Exception('Fast64 Not Installed!')

        self.mario_obj = bpy.data.objects['mario_geo'] # Don't get the evaluated copy so we can set properties on it in the scene
        self.start_time = time.perf_counter()
        self.find_floor_handler = FindFloorHandlerType(self.create_find_floor_handler())
        self.find_ceil_handler = FindFloorHandlerType(self.create_find_ceil_handler())
        self.find_wall_collisions_handler = FindWallsHandlerType(self.create_find_wall_collisions_handler())
        
        if not _t :
            _t = threading.Thread(target=worker)
            _t.daemon = True
            _t.start()
        
        libmario.init(self.find_floor_handler, self.find_ceil_handler, self.find_wall_collisions_handler)

        mario_pos = Vec3f(self.mario_obj.location.x * 100, self.mario_obj.location.z * 100, -self.mario_obj.location.y * 100)
        mario_rot = Vec3f(math.degrees(self.mario_obj.rotation_euler[0]), math.degrees(self.mario_obj.rotation_euler[1]), math.degrees(self.mario_obj.rotation_euler[2]))

        libmario.setMarioPosition(mario_pos)
        libmario.setMarioRotation(mario_rot)

        wm = context.window_manager
        self._timer = wm.event_timer_add(0.033333, window=context.window)
        wm.modal_handler_add(self)

        return {'RUNNING_MODAL'}
	
    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)

    def modal(self, context, event):
        global _t
        global events
        
        if event.type in {'RIGHTMOUSE', 'ESC'}:
            self.cancel(context)
            return {'CANCELLED'}

        self.process_updates(context)
        cur_time = time.perf_counter()


        if event.type in {'MOUSEMOVE','TIMER'}:
            if (cur_time - self.start_time) > (self.last_frame * 0.033333):
                self.last_frame += 1
                while len(events) > 0 :
                    for event in events[0]:
                        if event.code == "ABS_X":
                            self.stick_x = float(event.state) / 32768.0
                        elif event.code == "ABS_Y":
                            self.stick_y = float(event.state) / 32768.0
                        elif event.code == "ABS_RX":
                            gpd_input = "Right Stick X"
                        elif event.code == "ABS_RY":
                            gpd_input = "Right Stick Y"
                        elif event.code == "BTN_SOUTH":
                            if event.state == 1:
                                self.buttons |= A_BUTTON
                            else:
                                self.buttons &= ~A_BUTTON
                        elif event.code == "BTN_WEST":
                            if event.state == 1:
                                self.buttons |= B_BUTTON
                            else:
                                self.buttons &= ~B_BUTTON
                        elif event.code == "ABS_Z":
                            if event.state == 255:
                                self.buttons |= Z_TRIG
                            else:
                                self.buttons &= ~Z_TRIG
                        elif event.code != "SYN_REPORT":
                            print(event.code + ':' + str(event.state))
                    events.pop(0)

                if (self.stick_x * self.stick_x + self.stick_y * self.stick_y) < (0.05 * 0.05):
                    self.stick_x = 0.0
                    self.stick_y = 0.0
                
                libmario.step(self.buttons, c_float(self.stick_x), c_float(self.stick_y))
                pos = Vec3f()
                vel = Vec3f()
                rot = Vec3f()
                scale = Vec3f()
                torso_rot = Vec3f()
                libmario.getMarioPosition(pos)
                libmario.getMarioVelocity(vel)
                libmario.getMarioRotation(rot)
                libmario.getMarioScale(scale)
                libmario.getMarioTorsoRotation(torso_rot)
                anim_data = AnimData()
                libmario.getMarioAnimData(byref(anim_data))

                try:
                    pose = self.mario_obj.pose
                except ReferenceError:
                    self.mario_obj = bpy.data.objects['mario_geo']
                    pose = self.mario_obj.pose

                self.mario_obj.pose.bones['002-rotate'].rotation_euler[0] = math.radians(torso_rot[0])
                self.mario_obj.pose.bones['002-rotate'].rotation_euler[1] = math.radians(torso_rot[1])
                self.mario_obj.pose.bones['002-rotate'].rotation_euler[2] = math.radians(torso_rot[2])

                self.mario_obj.pose.bones[bone_names[0]].location[0] = float(anim_data.root_translation[0]) / 2.4 / 189.0
                self.mario_obj.pose.bones[bone_names[0]].location[1] = float(anim_data.root_translation[1]) / 2.4 / 189.0
                self.mario_obj.pose.bones[bone_names[0]].location[2] = float(anim_data.root_translation[2]) / 2.4 / 189.0

                for bone_index, bone_name in enumerate(bone_names):
                    self.mario_obj.pose.bones[bone_name].rotation_euler[0] = math.radians(anim_data.bone_rotations[bone_index][0])
                    self.mario_obj.pose.bones[bone_name].rotation_euler[1] = math.radians(anim_data.bone_rotations[bone_index][1])
                    self.mario_obj.pose.bones[bone_name].rotation_euler[2] = math.radians(anim_data.bone_rotations[bone_index][2])

                self.mario_obj.location.x = pos[0] / 100.0
                self.mario_obj.location.y = -pos[2] / 100.0
                self.mario_obj.location.z = pos[1] / 100.0

                self.mario_obj.rotation_euler[0] = math.radians(rot[0])
                self.mario_obj.rotation_euler[1] = math.radians(rot[2])
                self.mario_obj.rotation_euler[2] = math.radians(rot[1])

                self.mario_obj.scale[0] = scale[0]
                self.mario_obj.scale[1] = scale[2]
                self.mario_obj.scale[2] = scale[1]
    
        return {'PASS_THROUGH'}

def register():
    bpy.utils.register_class(MarioTester)


def unregister():
    bpy.utils.unregister_class(MarioTester)
