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

from .inputs import *
import time
import threading
import sys
import os
import math
from ctypes import *

import bpy

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

libmario.init.restype = None
libmario.init.artypes = []

libmario.step.restype = None
libmario.step.artypes = [c_int32, c_float, c_float]

libmario.getMarioPosition.restype = None
libmario.getMarioPosition.artypes = [Vec3f]

libmario.getMarioVelocity.restype = None
libmario.getMarioVelocity.artypes = [Vec3f]

libmario.getMarioRotation.restype = None
libmario.getMarioRotation.artypes = [Vec3f]

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

    def execute(self, context):
        global _t
        global events
        
        self.mario_obj = bpy.data.objects['mario_geo']

        self.start_time = time.perf_counter()
        
        if not _t :
            _t = threading.Thread(target=worker)
            _t.daemon = True
            _t.start()
        
        libmario.init()

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

        cur_time = time.perf_counter()

        if (cur_time - self.start_time) > (self.last_frame * 0.033333):
            self.last_frame += 1

            if event.type == 'TIMER':
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
                
                libmario.step(self.buttons, c_float(self.stick_x), c_float(self.stick_y))
                pos = Vec3f()
                vel = Vec3f()
                rot = Vec3f()
                libmario.getMarioPosition(pos)
                libmario.getMarioVelocity(vel)
                libmario.getMarioRotation(rot)
                anim_data = AnimData()
                libmario.getMarioAnimData(byref(anim_data))

                self.mario_obj.pose.bones[bone_names[0]].location[0] = float(anim_data.root_translation[0]) / 189.0
                self.mario_obj.pose.bones[bone_names[0]].location[1] = float(anim_data.root_translation[1]) / 189.0
                self.mario_obj.pose.bones[bone_names[0]].location[2] = float(anim_data.root_translation[2]) / 189.0

                for bone_index, bone_name in enumerate(bone_names):
                    self.mario_obj.pose.bones[bone_name].rotation_euler[0] = math.radians(anim_data.bone_rotations[bone_index][0])
                    self.mario_obj.pose.bones[bone_name].rotation_euler[1] = math.radians(anim_data.bone_rotations[bone_index][1])
                    self.mario_obj.pose.bones[bone_name].rotation_euler[2] = math.radians(anim_data.bone_rotations[bone_index][2])
                # for bone_index in range(1):
                #     bone_name = bone_names[bone_index]
                #     self.mario_obj.pose.bones[bone_name].rotation_euler[0] = math.radians(anim_data.bone_rotations[bone_index][0])
                #     self.mario_obj.pose.bones[bone_name].rotation_euler[1] = math.radians(anim_data.bone_rotations[bone_index][1])
                #     self.mario_obj.pose.bones[bone_name].rotation_euler[2] = math.radians(anim_data.bone_rotations[bone_index][2])

                self.mario_obj.location.x = pos[0] / 100.0
                self.mario_obj.location.y = -pos[2] / 100.0
                self.mario_obj.location.z = pos[1] / 100.0

                self.mario_obj.rotation_euler[0] = math.radians(rot[0])
                self.mario_obj.rotation_euler[1] = math.radians(rot[2])
                self.mario_obj.rotation_euler[2] = math.radians(rot[1])
    
        return {'PASS_THROUGH'}

def register():
    bpy.utils.register_class(MarioTester)


def unregister():
    bpy.utils.unregister_class(MarioTester)
