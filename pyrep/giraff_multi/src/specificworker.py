#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2020 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from genericworker import *
import time
from pyrep import PyRep
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
import numpy as np
import numpy_indexed as npi
import cv2
import itertools as it
from threading import Thread, Event
import queue
from math import *

class TimeControl:
    def __init__(self, period_):
        self.counter = 0
        self.start = time.time()  # it doesn't exist yet, so initialize it
        self.start_print = time.time()  # it doesn't exist yet, so initialize it
        self.period = period_

    def wait(self):
        elapsed = time.time() - self.start
        if elapsed < self.period:
            time.sleep(self.period - elapsed)
        self.start = time.time()
        self.counter += 1
        if time.time() - self.start_print > 1:
            print("Giraff PyRep - Freq -> ", self.counter, "Hz")
            self.counter = 0
            self.start_print = time.time()


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        SCENE_FILE = params["scene_file"]
        print("SCENE_FILE", SCENE_FILE)
        self.WITH_BILL = False
        if "bill" in SCENE_FILE:
            self.WITH_BILL = True

        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()

        # robots
        self.robots = dict()
        try:
            i = 0
            while True:
                self.robots[i] = {}
                robot_name = "/Giraff[" + str(i) + "]"
                self.robots[i]["shape"] = Shape(robot_name)
                self.robots[i]["id"] = i
                self.robots[i]["name"] = robot_name
                self.robots[i]["left_wheel"] = Joint(robot_name + "/Pioneer_p3dx_leftMotor")
                self.robots[i]["right_wheel"] = Joint(robot_name + "/Pioneer_p3dx_rightMotor")
                self.robots[i]["radius"] = 100  # wheel radius in mm
                self.robots[i]["semi_width"] = 165  # axle semi width in mm
                self.robots[i]["bState_write"] = RoboCompGenericBase.TBaseState()
                self.robots[i]["bState_read"] = RoboCompGenericBase.TBaseState()
                self.robots[i]["ldata_write"] = []
                self.robots[i]["ldata_read"] = []
                self.robots[i]["speed"] = [0, 0]
                i += 1
        except:
            del self.robots[i]
            print("Read ", i-1, "robots")
            #print(self.robots)

        # laser thread
        # self.laser_read_queue = queue.Queue(1)
        # self.event = Event()
        # self.laser_read_thread = Thread(target=self.laser_thread, args=[self.event], name="laser_read_queue", daemon=True)
        # self.laser_read_thread.start()

        # JoyStick
        self.joystick_newdata = []
        self.last_received_data_time = 0

        # Eye pan motor
        # self.eye_motor = Joint("camera_joint")
        # self.eye_new_pos = None

    def compute(self):
        tc = TimeControl(0.05)
        while True:
            self.pr.step()
            self.read_robot_pose()
            self.move_robot()
            self.read_laser_raw()
            #self.read_cameras([self.tablet_camera_name, self.top_camera_name])
            #self.read_people()
            self.read_joystick()
            #self.move_eye()
            tc.wait()

    ###########################################
    ### LASER get and publish laser data
    ###########################################
    def laser_thread(self, event: Event):
        while(True):
            for robot in self.robots.values():
                data = self.pr.script_call('get_depth_data@' + robot["name"] + '/Hokuyo', 1)
                if len(data[1]) > 0:
                    robot["ldata_write"] = []
                    for x, y, z in self.grouper(data[1], 3):                      # extract non-intersecting groups of 3
                        robot["ldata_write"].append(RoboCompLaser.TData(-np.arctan2(y, x), np.linalg.norm([x, y])*1000.0))

                    del robot["ldata_write"][-7:]
                    del robot["ldata_write"][:7]

                    robot["ldata_read"], robot["ldata_write"] = robot["ldata_write"], robot["ldata_read"]
            self.laser_read_queue.put(True)

    def read_laser_raw(self):
        for robot in self.robots.values():
            data = self.pr.script_call('get_depth_data@' + robot["name"] + '/Hokuyo', 1)
            if len(data[1]) > 0:
                robot["ldata_write"] = []
                for x, y, z in self.grouper(data[1], 3):                      # extract non-intersecting groups of 3
                    robot["ldata_write"].append(RoboCompLaserMulti.TData(-np.arctan2(y, x), np.linalg.norm([x, y])*1000.0))
                del robot["ldata_write"][-7:]
                del robot["ldata_write"][:7]

                robot["ldata_read"], robot["ldata_write"] = robot["ldata_write"], robot["ldata_read"]

    def read_laser(self):
        data = self.pr.script_call("get_depth_data@Hokuyo", 1)
        if len(data[1]) > 0:
            self.hokuyo = Shape("Hokuyo")
            h_pos = self.hokuyo.get_position()
            polar = np.zeros(shape=(int(len(data[1])/3), 2))
            i = 0
            for x, y, z in self.grouper(data[1], 3):                      # extract non-intersecting groups of 3
                # translate to the robot center
                #x += h_pos[0]
                #y += h_pos[1]
                polar[i] = [-np.arctan2(y, x), np.linalg.norm([x, y])]    # add to list in polar coordinates
                i += 1

            angles = np.linspace(-np.radians(120), np.radians(120), 360)  # create regular angular values
            positions = np.searchsorted(angles, polar[:, 0])  # list of closest position in polar for each laser measurement
            self.ldata_write = [RoboCompLaser.TData(a, 0) for a in angles]  # create empty 240 angle array
            pos, medians = npi.group_by(positions).median(polar[:, 1])  # group by repeated positions
            for p, m in it.zip_longest(pos, medians):  # fill the angles with measures
                if p < len(self.ldata_write):
                    self.ldata_write[p].dist = int(m * 1000)  # to millimeters
            if self.ldata_write[0] == 0:
               self.ldata_write[0] = 200  # half robot width
            del self.ldata_write[-3:]
            del self.ldata_write[:3]
            for i in range(1, len(self.ldata_write)):
               if self.ldata_write[i].dist == 0:
                   self.ldata_write[i].dist = self.ldata_write[i - 1].dist


            self.ldata_read, self.ldata_write = self.ldata_write, self.ldata_read

            # try:
            #     self.laserpub_proxy.pushLaserData(self.ldata_read)
            # except Ice.Exception as e:
            #     print(e)

    def grouper(self, inputs, n, fillvalue=None):
        iters = [iter(inputs)] * n
        return it.zip_longest(*iters, fillvalue=fillvalue)

    ###########################################
    ### CAMERAS get and publish cameras data
    ###########################################
    def read_cameras(self, camera_names):
         if self.tablet_camera_name in camera_names:  # RGB not-rotated
            cam = self.cameras_write[self.tablet_camera_name]
            image_float = cam["handle"].capture_rgb()   #TODO: CAMBIAR PARA COGER DIRECTAMENTE EN BYTES!!
            image = cv2.normalize(src=image_float, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                  dtype=cv2.CV_8U)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            cam["rgb"] = RoboCompCameraRGBDSimple.TImage(cameraID=cam["id"],
                                                         width=cam["width"],
                                                         height=cam["height"],
                                                         depth=3,
                                                         focalx=cam["focalx"],
                                                         focaly=cam["focaly"],
                                                         alivetime=int(time.time()*1000),
                                                         period=50,  # ms
                                                         image=image.tobytes(),
                                                         compressed=False)

            cam["is_ready"] = True

         if self.omni_camera_rgb_name in camera_names:  # RGB not-rotated
             cam = self.cameras_write[self.omni_camera_rgb_name]
             image_float = cam["handle"].capture_rgb()
             image = cv2.normalize(src=image_float, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                   dtype=cv2.CV_8U)
             image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
             cam["rgb"] = RoboCompCameraRGBDSimple.TImage(cameraID=cam["id"],
                                                          width=cam["width"],
                                                          height=cam["height"],
                                                          depth=3,
                                                          focalx=cam["focalx"],
                                                          focaly=cam["focaly"],
                                                          alivetime=int(time.time()*1000),
                                                          period=50,  # ms
                                                          image=image.tobytes(),
                                                          compressed=False)

             cam["is_ready"] = True

         if self.omni_camera_depth_name in camera_names:  # RGB not-rotated
             cam = self.cameras_write[self.omni_camera_depth_name]
             image_float = cam["handle"].capture_rgb()
             image = cv2.normalize(src=image_float, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                   dtype=cv2.CV_8U)
             image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
             cam["rgb"] = RoboCompCameraRGBDSimple.TImage(cameraID=cam["id"],
                                                          width=cam["width"],
                                                          height=cam["height"],
                                                          depth=3,
                                                          focalx=cam["focalx"],
                                                          focaly=cam["focaly"],
                                                          alivetime=int(time.time()*1000),
                                                          period=50,  # ms
                                                          image=image.tobytes(),
                                                          compressed=False)

             cam["is_ready"] = True

         if self.top_camera_name in camera_names:  # RGBD rotated
            cam = self.cameras_write[self.top_camera_name]
            image_float = cam["handle"].capture_rgb()
            image = cv2.normalize(src=image_float, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                   dtype=cv2.CV_8U)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            depth = cam["handle"].capture_depth(True)
            depth = np.frombuffer(depth, dtype=np.float32).reshape((cam["height"], cam["width"]))
            depth = cv2.rotate(depth, cv2.ROTATE_90_COUNTERCLOCKWISE)
            # we change width and height here to follow the rotation operation
            cam["depth"] = RoboCompCameraRGBDSimple.TDepth( cameraID=cam["id"],
                                                            width=cam["height"],  # cambiados
                                                            height=cam["width"],
                                                            focalx=cam["focaly"],
                                                            focaly=cam["focalx"],
                                                            alivetime=int(time.time()*1000),
                                                            period=50, # ms
                                                            depthFactor=1.0,
                                                            depth=depth.tobytes())
            cam["rgb"] = RoboCompCameraRGBDSimple.TImage( cameraID=cam["id"],
                                                          width=cam["height"],          #cambiados
                                                          height=cam["width"],
                                                          depth=3,
                                                          focalx=cam["focaly"],
                                                          focaly=cam["focalx"],
                                                          alivetime=int(time.time()*1000),
                                                          period=50,  # ms
                                                          image=image.tobytes(),
                                                          compressed=False)
            cam["is_ready"] = True

         self.cameras_write, self.cameras_read = self.cameras_read, self.cameras_write

    ###########################################
    ### JOYSITCK read and move the robot
    ###########################################
    def read_joystick(self):
        if self.joystick_newdata:  # and (time.time() - self.joystick_newdata[1]) > 0.1:
            datos = self.joystick_newdata[0]
            if not int(datos.id) in self.robots.keys():
                print("Robot ", datos.id, "not found")
                return
            adv = 0.0
            rot = 0.0
            bill_advance = 0.0
            bill_rotate = 0.0
            for x in datos.axes:
                if x.name == "advance":
                    adv = x.value if np.abs(x.value) > 10 else 0
                if x.name == "rotate" or x.name == "turn":
                    rot = x.value if np.abs(x.value) > 0.01 else 0
                if x.name == "bill_advance":
                    bill_advance = x.value if np.abs(x.value) > 0.05 else 0
                    print("bill_advance ", bill_advance)
                if x.name == "bill_rotate":
                    bill_rotate = x.value if np.abs(x.value) > 0.05 else 0

            self.convert_base_speed_to_motors_speed(int(datos.id), adv, rot)

            #print("Joystick ", [adv, rot], converted)
            self.joystick_newdata = None
            self.last_received_data_time = time.time()

    def convert_base_speed_to_motors_speed(self, idr: int, adv: float, rot: float):
        left_vel = (adv + self.robots[idr]["semi_width"] * rot) / self.robots[idr]["radius"]
        right_vel = (adv - self.robots[idr]["semi_width"] * rot) / self.robots[idr]["radius"]
        self.robots[idr]["left_wheel"].set_joint_target_velocity(left_vel)
        self.robots[idr]["right_wheel"].set_joint_target_velocity(right_vel)


    ###########################################
    ### ROBOT POSE get and publish robot position
    ###########################################
    def read_robot_pose(self):
        for robot in self.robots.values():
            pose = robot["shape"].get_position()
            rot = robot["shape"].get_orientation()
            linear_vel, ang_vel = robot["shape"].get_velocity()

            isMoving = np.abs(linear_vel[0]) > 0.01 or np.abs(linear_vel[1]) > 0.01 or np.abs(ang_vel[2]) > 0.01
            robot["bState_write"] = RoboCompGenericBase.TBaseState( x=pose[0] * 1000,
                                                                    z=pose[1] * 1000,
                                                                    alpha=rot[2],
                                                                    advVx=linear_vel[0] * 1000,
                                                                    advVz=linear_vel[1] * 1000,
                                                                    rotV=ang_vel[2],
                                                                    isMoving=isMoving)

            # swap
            robot["bState_write"], robot["bState_read"] = robot["bState_read"], robot["bState_write"]

    ###########################################
    ### MOVE ROBOT from Omnirobot interface
    ###########################################
    def move_robot(self):
        for robot in self.robots.values():
            if robot["speed"]:
                self.convert_base_speed_to_motors_speed(robot["id"], robot["speed"][0], robot["speed"][1])
            robot["speed"] = None

    ###########################################
    ### MOVE ROBOT from Omnirobot interface
    ###########################################
    def move_eye(self):
        if self.eye_new_pos:
            self.eye_motor.set_joint_position(self.eye_new_pos)  # radians
            self.eye_new_pos = None

    #################################################################
    ##################################################################################
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    ###################################################################################
    def JoystickAdapter_sendData(self, data):
        self.joystick_newdata = [data, time.time()]

    ##################################################################################
    #                       Methods for CameraRGBDSimple
    # ===============================================================================
    #
    # getAll
    #
    def CameraRGBDSimple_getAll(self, camera):
        if camera in self.cameras_read.keys() \
                and self.cameras_read[camera]["is_ready"] \
                and self.cameras_read[camera]["is_rgbd"]:
            return RoboCompCameraRGBDSimple.TRGBD(self.cameras_read[camera]["rgb"], self.cameras_read[camera]["depth"])
        else:
            e = RoboCompCameraRGBDSimple.HardwareFailedException()
            e.what = "No camera found with this name or with depth attributes: " + camera
            raise e

    #
    # getDepth
    #
    def CameraRGBDSimple_getDepth(self, camera):
        if camera in self.cameras_read.keys() \
                and self.cameras_read[camera]["is_ready"] \
                and self.cameras_read[camera]["has_depth"]:
            return self.cameras_read[camera]["depth"]
        else:
            e = RoboCompCameraRGBDSimple.HardwareFailedException()
            e.what = "No camera found with this name or with depth attributes: " + camera
            raise e

    #
    # getImage
    #
    def CameraRGBDSimple_getImage(self, camera):
        if camera in self.cameras_read.keys() and self.cameras_read[camera]["is_ready"]:
            return self.cameras_read[camera]["rgb"]
        else:
            e = RoboCompCameraRGBDSimple.HardwareFailedException()
            e.what = "No camera found with this name: " + camera
            raise e

    ##############################################
    ## Differentialbase
    #############################################

    #
    # correctOdometer
    #
    def DifferentialRobot_correctOdometer(self, x, z, alpha):
        pass

    #
    # getBasePose
    #
    def DifferentialRobot_getBasePose(self):
        if self.bState:
            x = self.bState.x
            z = self.bState.z
            alpha = self.bState.alpha
            return [x, z, alpha]
        else:
            return RoboCompGenericBase.TBaseState()

    #
    # getBaseState
    #
    def DifferentialRobot_getBaseState(self):
        if id in self.robots.keys():
            return self.robots[id]["bState_read"]
        else:
            return RoboCompGenericBase.TBaseState()

    #
    # resetOdometer
    #
    def DifferentialRobot_resetOdometer(self):
        pass

    #
    # setOdometer
    #
    def DifferentialRobot_setOdometer(self, state):
        pass

    #
    # setOdometerPose
    #
    def DifferentialRobot_setOdometerPose(self, x, z, alpha):
        pass

    #
    # setSpeedBase
    #
    def DifferentialRobot_setSpeedBase(self, advz, rot):
        self.robots[0]["speed"] = [advz, rot]
    #
    # stopBase
    #
    def DifferentialRobot_stopBase(self):
        pass

    def DifferentialRobotMulti_setSpeedBase(self, id, advz, rot):
        if id in self.robots.keys():
            self.robots[id]["speed"] = [advz, rot]

    def DifferentialRobotMulti_getBaseState(self, id):
        if id in self.robots.keys():
            return self.robots[id]["bState_read"]
        else:
            return RoboCompGenericBase.TBaseState()

    # ===================================================================
    # CoppeliaUtils
    # ===================================================================
    def CoppeliaUtils_addOrModifyDummy(self, type, name, pose):
        if not Dummy.exists(name):
            dummy = Dummy.create(0.1)
            # one color for each type of dummy
            if type == RoboCompCoppeliaUtils.TargetTypes.Info:
                pass
            if type == RoboCompCoppeliaUtils.TargetTypes.Hand:
                pass
            if type == RoboCompCoppeliaUtils.TargetTypes.HeadCamera:
                pass
            dummy.set_name(name)
        else:
            dummy = Dummy(name)
            parent_frame_object = None
            if type == RoboCompCoppeliaUtils.TargetTypes.HeadCamera:
                parent_frame_object = Dummy("viriato_head_camera_pan_tilt")
            # print("Coppelia ", name, pose.x/1000, pose.y/1000, pose.z/1000)
            dummy.set_position([pose.x / 1000., pose.y / 1000., pose.z / 1000.], parent_frame_object)
            dummy.set_orientation([pose.rx, pose.ry, pose.rz], parent_frame_object)

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getFullPose method from FullPoseEstimation interface
    #
    def FullPoseEstimation_getFullPoseEuler(self):
        return self.robot_full_pose_read

    def FullPoseEstimation_getFullPoseMatrix(self): # NO USAR
        t = self.tm.get_transform("origin", "robot")
        m = RoboCompFullPoseEstimation.FullPoseMatrix()
        m.m00 = t[0][0]
        m.m01 = t[0][1]
        m.m02 = t[0][2]
        m.m03 = t[0][3]
        m.m10 = t[1][0]
        m.m11 = t[1][1]
        m.m12 = t[1][2]
        m.m13 = t[1][3]
        m.m20 = t[2][0]
        m.m21 = t[2][1]
        m.m22 = t[2][2]
        m.m23 = t[2][3]
        m.m30 = t[3][0]
        m.m31 = t[3][1]
        m.m32 = t[3][2]
        m.m33 = t[3][3]
        return m

    #
    # IMPLEMENTATION of setInitialPose method from FullPoseEstimation interface
    #
    def FullPoseEstimation_setInitialPose(self, x, y, z, rx, ry, rz):

        # should move robot in Coppelia to designated pose
        self.tm.add_transform("origin", "world",
                               pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([rx, ry, rz]), [x, y, z])
        )

    #
    # IMPLEMENTATION of getAllSensorDistances method from Ultrasound interface
    #
    def Ultrasound_getAllSensorDistances(self):
        ret = RoboCompUltrasound.SensorsState()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getAllSensorParams method from Ultrasound interface
    #
    def Ultrasound_getAllSensorParams(self):
        ret = RoboCompUltrasound.SensorParamsList()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getBusParams method from Ultrasound interface
    #
    def Ultrasound_getBusParams(self):
        ret = RoboCompUltrasound.BusParams()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getSensorDistance method from Ultrasound interface
    #
    def Ultrasound_getSensorDistance(self, sensor):
        ret = int()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getSensorParams method from Ultrasound interface
    #
    def Ultrasound_getSensorParams(self, sensor):
        ret = RoboCompUltrasound.SensorParams()
        #
        # write your CODE here
        #
        return ret

    # ===================================================================
    # ===================================================================
    #
    # IMPLEMENTATION of getRSSIState method from RSSIStatus interface
    #
    def RSSIStatus_getRSSIState(self):
        ret = RoboCompRSSIStatus.TRSSI()
        ret.percentage = 100;
        return ret

    #
    # IMPLEMENTATION of getBatteryState method from BatteryStatus interface
    #
    def BatteryStatus_getBatteryState(self):
        ret = RoboCompBatteryStatus.TBattery()
        ret.percentage = 100
        return ret
    #
    #######################################################
    #### Laser
    #######################################################
    #
    # getLaserAndBStateData
    #
    def Laser_getLaserAndBStateData(self):
        bState = RoboCompGenericBase.TBaseState()
        return self.ldata_read, bState

   #
   # getLaserConfData
   #

    def Laser_getLaserConfData(self):
        ret = RoboCompLaser.LaserConfData()
        return ret

    #
    # getLaserData
    #

    def Laser_getLaserData(self):
        return self.ldata_read

 # ===================================================================
    # IMPLEMENTATION of getMotorParams method from JointMotorSimple interface
    # ===================================================================
    def LaserMulti_getLaserData(self, robotid):
        if robotid in self.robots.keys():
            return self.robots[robotid]["ldata_read"]

    def JointMotorSimple_getMotorParams(self, motor):
        ret = RoboCompJointMotorSimple.MotorParams()
        return ret

    #
    # IMPLEMENTATION of getMotorState method from JointMotorSimple interface
    #
    def JointMotorSimple_getMotorState(self, motor):
        if motor == "tablet_motor":
            ret = RoboCompJointMotorSimple.MotorState(self.tablet_motor.get_joint_position())  # radians
        elif motor == "eye_motor":
            ret = RoboCompJointMotorSimple.MotorState(self.eye_motor.get_joint_position())  # radians
        return ret

    #
    # IMPLEMENTATION of setPosition method from JointMotorSimple interface
    #
    def JointMotorSimple_setPosition(self, name, goal):
        print("JointMotorSimple_setPosition: ", name, goal)
        # check position limits -10 to 80
        if name == "tablet":
            self.tablet_new_pos = goal.position
        elif name == "eye":
            self.eye_new_pos = goal.position
        else: print("Unknown motor name", name)

    #
    # IMPLEMENTATION of setVelocity method from JointMotorSimple interface
    #
    def JointMotorSimple_setVelocity(self, name, goal):
        pass

    #
    # IMPLEMENTATION of setZeroPos method from JointMotorSimple interface
    #
    def JointMotorSimple_setZeroPos(self, name):

        #
        # write your CODE here
        #
        pass


   # =============== Methods for Component Implements ==================
    #
    # IMPLEMENTATION of getImage method from CameraSimple interface
    #
    def CameraSimple_getImage(self):
        camera = self.tablet_camera_name
        if camera in self.cameras_read.keys() \
                and self.cameras_read[camera]["is_ready"]\
                and not self.cameras_read[camera]["is_rgbd"]:
                    return self.cameras_read[camera]["rgb"]
        else:
            e = RoboCompCameraSimple.HardwareFailedException()
            e.what = "No (no RGBD) camera found with this name: " + camera
            raise e
    # ===================================================================
    # ===================================================================
    #
    # IMPLEMENTATION of getPose method from BillCoppelia interface
    #
    def BillCoppelia_getPose(self):
        ret = RoboCompBillCoppelia.Pose()
        bill = Dummy("/Bill/Bill")
        #bill = Dummy("Bill")
        pos = bill.get_position()
        print(pos)
        ret.x = pos[0] * 1000.0
        ret.y = pos[1] * 1000.0
        linear_vel, ang_vel = bill.get_velocity()
        ret.vx = linear_vel[0] * 1000.0
        ret.vy = linear_vel[1] * 1000.0
        ret.vo = ang_vel[2]
        ret.orientation = bill.get_orientation()[2]
        return ret

    #
    # IMPLEMENTATION of setSpeed method from BillCoppelia interface
    #
    def BillCoppelia_setSpeed(self, adv, rot):
        pass

    #
    # IMPLEMENTATION of setTarget method from BillCoppelia interface
    #
    def BillCoppelia_setTarget(self, tx, ty):
        bill_target = Dummy("Bill_goalDummy")
        current_pos = bill_target.get_position()
        bill_target.set_position([tx/1000.0, ty/1000.0, current_pos[2]])

    # ===================================================================


