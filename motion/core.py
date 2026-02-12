#!/usr/bin/python3

#
#   Developer : Mikhail Filchenkov (m.filchenkov@gmail.com)
#   All rights reserved. Copyright (c) 2025 Applied Robotics.
#

import os
import motorcortex
from motorcortex.request import Request
from math import *
from .robot_control.motion_program import Waypoint, MotionProgram, PoseTransformer
from .robot_control.robot_command import RobotCommand
from .robot_control.system_defs import InterpreterStates, States, ModeCommands, Modes
from .system_def import Path, JoyVelicity
from concurrent.futures import Future, ThreadPoolExecutor
import logging  
import re
from importlib.resources import files
import socket
import time
import datetime
import threading
import requests
from typing import List, Optional

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.propagate = False

if not logger.handlers:
    handler = logging.StreamHandler()
    formatter = logging.Formatter('[%(levelname)s] %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)

class MotionSpecWarningFilter(logging.Filter):
    def filter(self, record):
        if record.levelno == logging.WARNING:
            message = record.getMessage()
            if re.search(r"Module motion_spec(_v1)? already exists", message):
                return False
        return True
    
def setup_logging():
    root_logger = logging.getLogger()
    root_logger.addFilter(MotionSpecWarningFilter())

    for handler in root_logger.handlers:
        handler.addFilter(MotionSpecWarningFilter())

class LedLamp(object):
    """
    Class represents a control led lamp of the robot ARM.
    Args:
        ip(str): IP-address robot ARM
        port(str): Port robot ARM

    """

    def __init__(self, ip='192.168.2.101', port=8890):
        self.__hostname = ip
        self.__port = port
        self.timeout = 0.2
        self._lock = threading.RLock()
        
    def setLamp(self, status: str = "0000") -> bool:
        """
        Set light to the lamp.
            Args:
                state: status for each color

            Description:
                The status for the "state" variable is written in the sequence:
                  - 1111 to turn on all colors
                  - 0000 to turn off all colors 
            
                - The first digit corresponds (**1**000) -> blue color
                - The second digit corresponds (0**1**00) -> green color
                - The third digit corresponds (00**1**0) -> yellow color
                - The fourth digit corresponds (000**1**) -> red color
        
            Returns:
                bool: True if operation succeeded, False otherwise
        """
        with self._lock:
            if len(status) != 4 or not all(c in "01" for c in status):
                logger.error("The status must be a string of four characters containing only '0' or '1'")
                return False
            
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sock.settimeout(self.timeout)
                    sock.sendto(status.encode(), (self.__hostname, self.__port))
                    logger.info(f"Led lamp was set: {status}")
                    return True
            except socket.timeout:
                logger.error(f"Timeout setting lamp after {self.timeout} seconds")
                return False
            except Exception as e:
                logger.exception(f"Error when sending UDP: {e}")
                return False
            
class SmartCamera(object):
    """
    Class represents working with the smart camera.
    Args:
        ip(str): IP-address robot ARM
    """

    def __init__(self, ip='192.168.2.110', port=8080):
        self.__hostname = ip
        self.__port = port
        self.__url = f"http://{self.__hostname}:{self.__port}/status"

    def getPerson(self) -> int:
        """
        Description:
            0 - the person is missing
            1 - the person is present
        
        Returns:
            int: camera status of presence person 
        """
        status = -1
        try:
            response = requests.get(self.__url, timeout=5)

            if response.status_code == 200:
                data = response.json()
                status = data["status"]
            else:
                logger.warning("Host server not found")
            
            return int(status)

        except Exception as e:
            logger.error(f"Unexpected error during connection: Connection timeout")
            return int(status)

class RobotControl(object):
    """
    Class represents a control machine of the robot ARM.
    Args:
        ip(str): IP-address robot ARM
        port(str): Port robot ARM
        login(str): login robot ARM
        password(str): password robot ARM
        timeout(float): timeout for operations in seconds

    """
        
    def __init__(self, ip='192.168.2.100', port='5568:5567', login='*', password='*', timeout=0.4):
        self.__hostname = ip
        self.__port = port
        self.__login = login
        self.__password = password
        self.__timeout = timeout
        self.__pathCert = files("motion").joinpath("mcx.cert.pem")
        self.__certificate = [str(self.__pathCert)]
        self._executor = ThreadPoolExecutor(max_workers=5)
        self._futures = {}
        self._stop_event = threading.Event()
        self._connection_lock = threading.RLock()
        self._cache = {}
        self._cache_time = {}
        self._is_connected = False
        self._is_engaged = False
        setup_logging()
        
    def __del__(self):
        self._stop_event.set()
        self._executor.shutdown(wait=False)
        if hasattr(self, '__req'):
            try:
                self.__req.close()
            except:
                pass
        if hasattr(self, '__sub'):
            try:
                self.__sub.close()
            except:
                pass
    
    def _get_cached_parameter(self, path, max_age=0.1):
        """Get parameter with caching to reduce network requests"""
        current_time = time.time()
        if path in self._cache and current_time - self._cache_time[path] < max_age:
            return self._cache[path]
        
        try:
            value = self.__req.getParameter(path).get()
            self._cache[path] = value
            self._cache_time[path] = current_time
            return value
        except Exception as e:
            logger.error(f"Failed to get parameter {path}: {e}")
            if path in self._cache:
                return self._cache[path] 
            raise
    
    def _is_operation_allowed(self):
        """Check if robot is in a state that allows operations"""
        if not self._is_connected:
            logger.error("Robot is not connected")
            return False
                
        try:
            state = self.getRobotState()
            return True
        except Exception as e:
            logger.error(f"Failed to check robot state: {e}")
            return False
        
    def connect(self) -> bool:
        """
        Connect to the robot ARM
            Returns:
                bool: True if operation is completed, False if failed
            
        """
        with self._connection_lock:
            if self._is_connected:
                logger.info("Already connected to robot")
                return True
                
            parameter_tree = motorcortex.ParameterTree()
            self.__messageTypes = motorcortex.MessageTypes()
            try:
                self.__req, self.__sub = motorcortex.connect("wss://" + self.__hostname + ":" + self.__port,
                                                    self.__messageTypes, parameter_tree,
                                                    certificate=self.__certificate, timeout_ms=1000,
                                                    login=self.__login, password=self.__password)
                
                self.__robot = RobotCommand(self.__req, self.__messageTypes)
                self.__conveyer = Conveyer(self.__req)
                
                # Test conveyer control
                self.__conveyer.start()
                time.sleep(0.1)
                self.__conveyer.stop()
                
                self._is_connected = True
                logger.info("Robot ARM connected successfully")
                return True
                
            except RuntimeError as err:
                logger.error(f"Failed to connect to robot: {err}")
                return False
            except Exception as e:
                logger.error(f"Unexpected error during connection: {e}")
                return False
        
    def conveyer_start(self) -> bool:
        """
        Start motor to the conveyer
            Returns:
                bool: True if operation is completed, False if failed
        
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            result = self.__conveyer.start()
            self.__req.setParameter(Path.CONVEYER_PINS.value[0], 1).get()
            time.sleep(0.1)
            if result:
                logger.info("Conveyer started successfully")
            else:
                logger.warning("Conveyer failed to start")
            return result
        
        except Exception as e:
            logger.error(f"Error starting conveyer: {e}")
            return False
        
    def conveyer_stop(self) -> bool:
        """
        Stop motor to the conveyer
            Returns:
                bool: True if operation is completed, False if failed
        
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            result = self.__conveyer.stop()
            self.__req.setParameter(Path.CONVEYER_PINS.value[0], 0).get()
            time.sleep(0.1)
            if result:
                logger.info("Conveyer stopped successfully")
            else:
                logger.warning("Conveyer failed to stop")
            return result
        
        except Exception as e:
            logger.error(f"Error stopping conveyer: {e}")
            return False
        
    def engage(self) -> bool:
        """
        Engage motor to the robot ARM
            Returns:
                bool: True if operation is completed, False if failed
        
        """
        if not self._is_operation_allowed():
            return False

        def engageCallback():
            try:
                if self.__robot.engage():
                    self._is_engaged = True
                    time.sleep(self.__timeout)
                    logger.info("Robot is Engaged")
                    return True
                else:
                    logger.error("Failed to engage robot")
                    return False
            except RuntimeError as err:
                logger.error(f"Error engaging robot: {err}")
                return False
            except Exception as e:
                logger.error(f"Unexpected error engaging robot: {e}")
                return False
        
        future = self._executor.submit(engageCallback)
        self._futures['engage'] = future
        return True
    
    def disengage(self) -> bool:
        """
        Disengage motor to the robot ARM
            Returns:
                bool: True if operation is completed, False if failed
        
        """
        def disengageCallback():
            try:
                self.__robot.off()
                if hasattr(self, '__req'):
                    self.__req.close()
                if hasattr(self, '__sub'):
                    self.__sub.close()
                self._is_connected = False
                logger.info("Robot is Disengaged")

            except RuntimeError as err:
                logger.error(f"Error disengaging robot: {err}")
                return False
            except Exception as e:
                logger.error(f"Unexpected error disengaging robot: {e}")
                return False
        
        future = self._executor.submit(disengageCallback)
        self._futures['disengage'] = future
        return True

    def manualCartMode(self) -> bool:
        """
        Set manual cartesian mode for robot ARM.
            Returns:
                bool: True if operation is completed, False if failed
        
        """
        if not self._is_operation_allowed():
            return False
            
        def manualCartModeCallback():
            try:
                if self.__robot.manualCartMode():
                    time.sleep(self.__timeout)
                    logger.info("Cartesian mode activated")
                    return True
                else:
                    logger.error("Failed to activate cartesian mode")
                    return False
            except RuntimeError as err:
                logger.error(f"Error setting cartesian mode: {err}")
                return False
            except Exception as e:
                logger.error(f"Unexpected error setting cartesian mode: {e}")
                return False
            
        future = self._executor.submit(manualCartModeCallback)
        self._futures['manualCartMode'] = future
        return True
        
    def manualJointMode(self) -> bool:
        """
        Set manual joint mode for robot ARM.
            Returns:
                bool: True if operation is completed, False if failed
        
        """
        if not self._is_operation_allowed():
            return False
            
        def manualJointModeCallback():
            try:
                if self.__robot.manualJointMode():
                    time.sleep(self.__timeout)
                    logger.info("Joint mode activated")
                    return True
                else:
                    logger.error("Failed to activate joint mode")
                    return False
            except RuntimeError as err:
                logger.error(f"Error setting joint mode: {err}")
                return False
            except Exception as e:
                logger.error(f"Unexpected error setting joint mode: {e}")
                return False
        
        future = self._executor.submit(manualJointModeCallback)
        self._futures['manualJointMode'] = future
        return True

    def setJointVelocity(self, velocity=None) -> bool:
        """
        Robot ARM control in joint mode by joysticks.
            Args:
                velocity(list(double)): joint velocity (motor1, motor2, motor3, motor4, motor5, motor6)

            Returns:
                bool: True if operation is completed, False if failed
        
        """
        if not self._is_operation_allowed():
            return False
            
        if velocity is None:
            velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
        if len(velocity) != 6:
            logger.warning("The number of values doesn't correspond to the number of motors")
            return False

        try:
            actual_state = self._get_cached_parameter(Path.ROBOT_MODE.value).value[0]
            if actual_state == ModeCommands.GOTO_MANUAL_JOINT_MODE_E.value:
                cap_vel = self.__cap_velocity(velocity, JoyVelicity.MAX_JOY_VELOCITY_JOINT.value)
                result = self.__req.setParameter(Path.HOSTIN_JOINT_VELOCITY.value, cap_vel).get()
                return result is not None
            else:
                logger.info("Actual robot state isn't joint mode")
                return False
        except Exception as e:
            logger.error(f"Error setting joint velocity: {e}")
            return False
       
    def setLinearTrackVelocity(self, velocity=None) -> bool:
        """
        Robot Linear Track control in joint mode by joysticks.
            Args:
                velocity(list(double)): joint velocity
            Returns:
                bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        if velocity is None:
            velocity = [0.0]
            
        if len(velocity) != 1:
            logger.warning("The number of values doesn't correspond to the tool")
            return False

        try:
            actual_state = self._get_cached_parameter(Path.ROBOT_MODE.value).value[0]
            if actual_state == ModeCommands.GOTO_MANUAL_JOINT_MODE_E.value:
                cap_vel = self.__cap_velocity(velocity, JoyVelicity.MAX_JOY_VELOCITY_LINEARTRACK.value)
                result = self.__req.setParameter(Path.HOSTIN_LINEARTRACK_VELOCITY.value, cap_vel).get()
                return result is not None
            else:
                logger.info("Actual robot state isn't joint mode")
                return False
        except Exception as e:
            logger.error(f"Error setting linear track velocity: {e}")
            return False
        
    def setCartesianVelocity(self, velocity=None) -> bool:
        """
        Robot ARM control in cartesian mode by joysticks.
            Args:
                velocity(list(double)): cartesian velocity (x, y, z, rx, ry, rz)

            Returns:
                bool: True if operation is completed, False if failed
        
        """
        if not self._is_operation_allowed():
            return False
            
        if velocity is None:
            velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
        if len(velocity) != 6:
            logger.warning("The number of values doesn't correspond to the tool")
            return False

        try:
            actual_state = self._get_cached_parameter(Path.ROBOT_MODE.value).value[0]
            if actual_state == ModeCommands.GOTO_MANUAL_CART_MODE_E.value:
                cap_vel = self.__cap_velocity(velocity, JoyVelicity.MAX_JOY_VELOCITY_CARTESIAN.value)
                result = self.__req.setParameter(Path.HOSTIN_TOOL_VELOCITY.value, cap_vel).get()
                return result is not None
            else:
                logger.info("Actual robot state isn't cartesian mode")
                return False
        except Exception as e:
            logger.error(f"Error setting cartesian velocity: {e}")
            return False
    
    def moveToInitialPose(self) -> bool:
        """
        Automatic activate move to start robot ARM.
            Description:
                Used for the button
        """
        if not self._is_operation_allowed():
            return False
            
        def moveToStartCallback():
            try:
                motion_program = MotionProgram(self.__req, self.__messageTypes)    
                start_point = []
                start_point.append(Waypoint([radians(0), radians(0), radians(90), radians(0), radians(90), radians(0)]))
                motion_program.addMoveJ(start_point, 0.05, 0.1)

                motion_program.send("move_to_start_point").get() 

                if self.__robot.play(wait_time=0.25) == InterpreterStates.MOTION_NOT_ALLOWED_S.value:
                    if self.__robot.moveToStart(200):
                        logger.info("Robot move to start position")
                    else:
                        logger.warning('Failed to move to the start position')
                        
                motion_program.send("move_to_start_point").get() 
                self.__robot.play(wait_time=0.25)

                while not(self.__robot.play(wait_time=0.25) == InterpreterStates.PROGRAM_IS_DONE.value):
                    time.sleep(0.1)
                else:
                    logger.info('Robot is at the start position')

                self.__robot.reset()    
                self.__robot.semiAutoMode()
                
                return True
                
            except Exception as e:
                logger.error(f"Error in moveToStart: {str(e)}")
                return False
        
        future = self._executor.submit(moveToStartCallback)
        self._futures['moveToStart'] = future
        return True
        
    def activateMoveToStart(self) -> bool:
        """
        Manual activate move to start robot ARM.
            Description:
                Used for the hold button
        
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            logger.info("Robot move to start")
            result = self.__req.setParameter(Path.ACTIVATE_MOVE_TO_START.value, 1).get()
            return result is not None
        
        except Exception as e:
            logger.error(f"Error activating move to start: {e}")
            return False
        
    def addMoveToPointL(self, waypoint_list, velocity=0.1, acceleration=0.2,
                 rotational_velocity=3.18, rotational_acceleration=6.37,
                 ref_joint_coord_rad=[]) -> bool:
        """
        Adds a MoveL(Linear move) command to the program
        Args:
            waypoint_list(list(WayPoint)): a list of waypoints
            velocity(double): maximum velocity, m/sec
            acceleration(double): maximum acceleration, m/sec^2
            rotational_velocity(double): maximum joint velocity, rad/sec
            rotational_acceleration(double): maximum joint acceleration, rad/sec^2
            ref_joint_coord_rad: reference joint coordinates for the first waypoint
        Description:
            Waypoint([x, y, z, rx, ry, rz]) - the waypoint is set as the absolute position of the manipulator in meters
        Returns:
            bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            if self.getRobotMode() == Modes.AUTO_RUN_M.value:
                self.__robot.stop()

            def moveToPointLCallback():
                try:
                    motion_program = MotionProgram(self.__req, self.__messageTypes) 
                    motion_program.addMoveL(waypoint_list, velocity, acceleration,
                            rotational_velocity, rotational_acceleration,
                            ref_joint_coord_rad)
                    motion_program.send("move_to_point_l").get() 
                    time.sleep(self.__timeout)
                except Exception as e:
                    logger.error(f"Error in moveToPointL: {e}")
                    return False

            future = self._executor.submit(moveToPointLCallback)
            self._futures['moveToPointL'] = future
            return True

        except Exception as e:
            logger.error(f"Error adding moveToPointL: {e}")
            return False

    def addMoveToPointC(self, waypoint_list, angle, velocity=0.1, acceleration=0.2,
                 rotational_velocity=3.18, rotational_acceleration=6.37,
                 ref_joint_coord_rad=[]) -> bool:
        
        """
        Adds a MoveC(circular move) command to the program
        Args:
            waypoint_list(list(WayPoint)): a list of waypoints
            angle(double): rotation angle, rad
            velocity(double): maximum velocity, m/sec
            acceleration(double): maximum acceleration, m/sec^2
            rotational_velocity(double): maximum joint velocity, rad/sec
            rotational_acceleration(double): maximum joint acceleration, rad/sec^2
            ref_joint_coord_rad: reference joint coordinates for the first waypoint
        Description:
            Waypoint([x, y, z, rx, ry, rz]) - the waypoint is set as the absolute position of the manipulator 
            x, y, z in meters (example: 0.85, -0.191, 0.921)
            rx, ry, rz in radians (example: pi/2, 0, pi)
        Returns:
            bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            if self.getRobotMode() == Modes.AUTO_RUN_M.value:
                self.__robot.stop()

            def moveToPointCCallback():
                try:
                    motion_program = MotionProgram(self.__req, self.__messageTypes) 
                    motion_program.addMoveC(waypoint_list, angle, velocity, acceleration,
                            rotational_velocity, rotational_acceleration,
                            ref_joint_coord_rad)
                    motion_program.send("move_to_point_c").get() 
                    time.sleep(self.__timeout)
                except Exception as e:
                    logger.error(f"Error in moveToPointC: {e}")
                    return False

            future = self._executor.submit(moveToPointCCallback)
            self._futures['moveToPointC'] = future
            return True

        except Exception as e:
            logger.error(f"Error adding moveToPointC: {e}")
            return False

    def addMoveToPointJ(self, waypoint_list=None, rotational_velocity=pi/4, rotational_acceleration=pi/2) -> bool:
        """
        Adds MoveJ(Joint move) command to the program
        Args:
            waypoint_list(list(WayPoint)): a list of waypoints
            rotational_velocity(double): maximum joint velocity, rad/sec
            rotational_acceleration(double): maximum joint acceleration, rad/sec^2
        Description:
            Waypoint(0.0, 0.0, 1.57, 0.0, 1.57, 0.0) - the waypoint is set as the position of the motors in radians
        Returns:
            bool: True if operation is completed, False if failed

        """
        if not self._is_operation_allowed():
            return False
            
        if waypoint_list is None:
            waypoint_list = []
            
        try:
            if self.getRobotMode() == Modes.AUTO_RUN_M.value:
                self.__robot.stop()
            
            def moveToPointJCallback():
                try:
                    motion_program = MotionProgram(self.__req, self.__messageTypes) 
                    motion_program.addMoveJ(waypoint_list, rotational_velocity, rotational_acceleration)
                    motion_program.send("move_to_point_j").get() 
                    time.sleep(self.__timeout)
                except Exception as e:
                    logger.error(f"Error in moveToPointJ: {e}")
                    return False

            future = self._executor.submit(moveToPointJCallback)
            self._futures['moveToPointJ'] = future
            return True

        except Exception as e:
            logger.error(f"Error adding moveToPointJ: {e}")
            return False
    
    # def addLinearTrackMove(self, position: float = 0.0, robot_mode: str = "addMoveToPointJ", robot_pose: list = [radians(0.0), radians(0.0), radians(90.0), radians(0.0), radians(90.0), radians(0.0)]) -> bool:
    #     """
    #     Adding LinearTrack movement to an executable program
    #         Arg: 
    #             position(int): the position is set in meters 
    #         Returns:
    #             bool: True if operation is completed, False if failed
    #     """
    #     if not self._is_operation_allowed():
    #         return False
            
    #     try:
    #         def linearTrackMoveCallback():
    #             try:
    #                 motion_program = MotionProgram(self.__req, self.__messageTypes) 
    #                 # actualPose = self.getToolPosition()
    #                 if robot_mode == "addMoveToPointJ":
    #                     motion_program.addMoveJ([Waypoint(robot_pose)],
    #                         rotational_velocity=3.18, rotational_acceleration=6.37)
                        
    #                 if robot_mode == "addMoveToPointL":
    #                     motion_program.addMoveL([Waypoint(robot_pose)], velocity=0.1, acceleration=0.2,
    #                         rotational_velocity=3.18, rotational_acceleration=6.37,
    #                         ref_joint_coord_rad=[])


    #                 # motion_program.addMoveL([Waypoint([actualPose[0], actualPose[1], actualPose[2], actualPose[3], actualPose[4], actualPose[5]])], velocity=0.1, acceleration=0.2,
    #                 #         rotational_velocity=3.18, rotational_acceleration=6.37,
    #                 #         ref_joint_coord_rad=[])
    #                 motion_program.addLinearTrackMove(position)
    #                 motion_program.send("linear_track_move").get() 
    #                 time.sleep(self.__timeout)

    #             except Exception as e:
    #                 logger.error(f"Error in linearTrackMove: {e}")
    #                 return False

    #         future = self._executor.submit(linearTrackMoveCallback)
    #         self._futures['linearTrackMove'] = future
    #         return True
        
    #     except Exception as e:
    #         logger.error(f"Error adding linear track move: {e}")
    #         return False

    def addLinearTrackMove(self, position: float = 0.0) -> bool:
        """
        Adding LinearTrack movement to an executable program
            Arg: 
                position(int): the position is set in meters 
            Returns:
                bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            def linearTrackMoveCallback():
                try:
                    motion_program = MotionProgram(self.__req, self.__messageTypes) 
                    motion_program.addLinearTrackMove(position)
                    motion_program.send("linear_track_move").get() 
                    time.sleep(self.__timeout)

                except Exception as e:
                    logger.error(f"Error in linearTrackMove: {e}")
                    return False

            future = self._executor.submit(linearTrackMoveCallback)
            self._futures['linearTrackMove'] = future
            return True
        
        except Exception as e:
            logger.error(f"Error adding linear track move: {e}")
            return False
        
    def addToolState(self, value: int = 0) -> bool:
        """
        Adding Tool state to an executable program
            Arg: 
                value(int): set as 1/0
            Description: 
                When the **value** is 1, the tool will start working
                When the **value** is 0, the tool will stop working
            Returns:
                bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            def addGripperStateCallback():
                try:
                    if value == 0 or value == 1:
                        motion_program = MotionProgram(self.__req, self.__messageTypes) 
                        motion_program.addSetParameter(Path.TOOL_CMD.value, value)
                        motion_program.send("gripper_state").get()
                        time.sleep(self.__timeout) 
                    else:
                        logger.warning("Set value as 0/1")
                except Exception as e:
                    logger.error(f"Error setting tool state: {e}")
                    return False

            future = self._executor.submit(addGripperStateCallback)
            self._futures['addToolState'] = future
            return True
    
        except Exception as e:
            logger.error(f"Error adding tool state: {e}")
            return False
    
    def addWait(self, wait_time: float = 0.0) -> bool:
        """
        Adding wait to an executable program
            Arg: 
                time(float): the time is set in seconds
            Returns:
                bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            def addWaitCallback():
                try:
                    motion_program = MotionProgram(self.__req, self.__messageTypes) 
                    motion_program.addWait(wait_time)
                    motion_program.send("wait_programm").get() 
                    time.sleep(self.__timeout)
                except Exception as e:
                    logger.error(f"Error adding wait: {e}")
                    return False

            future = self._executor.submit(addWaitCallback)
            self._futures['addWait'] = future
            return True
        
        except Exception as e:
            logger.error(f"Error adding wait: {e}")
            return False
    
    def addConveyerState(self, value: int = 0) -> bool:
        """
        Adding pipeline state to an executable program
            Arg: 
                value(int): set as 1/0
            Description: 
                When the **value** is 1, the pipeline will start working
                When the **value** is 0, the pipeline will stop working
            Returns:
                bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            def addConveyerStateCallback():
                try:
                    if value == 0 or value == 1:
                        motion_program = MotionProgram(self.__req, self.__messageTypes) 
                        motion_program.addSetParameter(Path.CONVEYER_PINS.value[0], value)
                        motion_program.send("conveyer_state").get() 
                        time.sleep(self.__timeout)
                    else:
                        logger.warning("Set value as 0/1")
                except Exception as e:
                    logger.error(f"Error setting conveyer state: {e}")
                    return False

            future = self._executor.submit(addConveyerStateCallback)
            self._futures['addConveyerState'] = future
            return True
        
        except Exception as e:
            logger.error(f"Error adding conveyer state: {e}")
            return False

    def play(self) -> bool:
        """
        Is used for start executable programm
            Returns:
                bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            def playCallback():
                try:
                    logger.info("Robot program play")
                    play_result = self.__robot.play(wait_time=0.25)
                    if play_result == InterpreterStates.MOTION_NOT_ALLOWED_S.value:
                        logger.info("Robot is not in start position")
                        logger.info("Please hold down activateMoveToStart function")
                    else:
                        result = self.__req.setParameter(Path.TIME_SCALE_CMD.value, 1.0).get()
                        return result is not None
                except Exception as e:
                    logger.error(f"Error playing program: {e}")
                    return False

            future = self._executor.submit(playCallback)
            self._futures['play'] = future
            return True
        
        except Exception as e:
            logger.error(f"Error starting play: {e}")
            return False
        
    def pause(self)-> bool:
        """
        Is used for pause executable programm
            Returns:
                bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            logger.info("Robot program pause")
            result = self.__req.setParameter(Path.TIME_SCALE_CMD.value, 0.0).get()
            return result is not None
        except Exception as e:
            logger.error(f"Error pausing program: {e}")
            return False

    def stop(self) -> bool:
        """
        Is used for stop executable programm
            Returns:
                bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            logger.info("Robot program stop")
            result = self.__robot.stop()
            return result is not None
        except Exception as e:
            logger.error(f"Error stopping program: {e}")
            return False

    def reset(self) -> bool:
        """
        Is used for reset executable programm
            Returns:
                bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            logger.info("Robot program reset")
            result = self.__robot.reset()
            return result is not None
        except Exception as e:
            logger.error(f"Error resetting program: {e}")
            return False

    def toolON(self) -> bool:
        """
        Turns on the working tool
            Description:
                If there is a vacuum system, the suction cup is activated.
                In the presence of a gripping device, the grip is compressed.
            Returns:
                bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            logger.info("Tool ON")
            result = self.__req.setParameter(Path.TOOL_CMD.value, 1).get()
            return result is not None
        except Exception as e:
            logger.error(f"Error turning tool ON: {e}")
            return False
    
    def toolOFF(self) -> bool:
        """
        Turns off the working tool
            Description:
                If there is a vacuum system, the suction cup is disactivated.
                In the presence of a gripping device, the grip is unclenches.
            Returns:
                bool: True if operation is completed, False if failed
        """
        if not self._is_operation_allowed():
            return False
            
        try:
            logger.info("Tool OFF")
            result = self.__req.setParameter(Path.TOOL_CMD.value, 0).get()
            return result is not None
        except Exception as e:
            logger.error(f"Error turning tool OFF: {e}")
            return False
    
    def getRobotMode(self) -> Optional[int]:
        """
        Returns:
            int: actual robot mode or None if failed

        """
        if not self._is_connected:
            return None
            
        try:
            result = self._get_cached_parameter(Path.ROBOT_MODE.value)
            return result.value[0] if result else None
        except Exception as e:
            logger.error(f"Error getting robot mode: {e}")
            return None
    
    def getRobotState(self) -> Optional[int]:
        """
        Returns:
            int: actual robot state or None if failed

        """
        if not self._is_connected:
            return None
            
        try:
            result = self._get_cached_parameter(Path.ROBOT_STATE.value)
            actual_state = result.value[0] if result else None
            
            if actual_state in [6, 7]:
                self.__conveyer.stop()
                
            command_result = self._get_cached_parameter(Path.ROBOT_STATE_COMMAND.value)
            return command_result.value[0] if command_result else None
        except Exception as e:
            logger.error(f"Error getting robot state: {e}")
            return None
    
    def getActualStateOut(self) -> Optional[int]:
        """
        Returns:
            int: actual state of the interpreter or None if failed

        """
        if not self._is_connected:
            return None
            
        try:
            result = self._get_cached_parameter(Path.STATE_CMD.value)
            return result.value[0] if result else None
        except Exception as e:
            logger.error(f"Error getting actual state: {e}")
            return None
    
    def getMotorPositionTick(self) -> Optional[List[float]]:
        """
        Returns:
            list(float): actual encoder positions or None if failed
        """
        if not self._is_connected:
            return None
            
        try:
            positions = []
            for tick in Path.CURRENT_JOINT_POSE_TICK.value:
                result = self._get_cached_parameter(tick)
                positions.append(result.value[0] if result else 0.0)
            return positions
        except Exception as e:
            logger.error(f"Error getting motor positions (ticks): {e}")
            return None

    def getToolPosition(self) -> Optional[List[float]]:
        """
        Returns:
            list(float): actual tool positions or None if failed
        """
        if not self._is_connected:
            return None
            
        try:
            result = self._get_cached_parameter(Path.CURRENT_TOOL_POSE.value)
            return list(result.value) if result else None
        except Exception as e:
            logger.error(f"Error getting tool position: {e}")
            return None
        
    def getToolState(self) -> Optional[int]:
        """
        Returns:
            list(float): actual tool positions or None if failed
        """
        if not self._is_connected:
            return None
            
        try:
            result = self._get_cached_parameter(Path.TOOL_CMD.value)
            return int(result.value[0]) if result else None
        except Exception as e:
            logger.error(f"Error getting tool state: {e}")
            return None

    def getMotorPositionRadians(self) -> Optional[List[float]]:
        """
        Returns:
            list(float): actual motor positions in radians or None if failed
        """
        if not self._is_connected:
            return None
            
        try:
            result = self._get_cached_parameter(Path.CURRENT_JOINT_POSE_RADIANS.value)
            return list(result.value[:6]) if result else None
        except Exception as e:
            logger.error(f"Error getting motor positions (radians): {e}")
            return None
        
    def getLinearTrackPosition(self) -> Optional[List[float]]:
        """
        Returns:
            list(float): actual motor positions in radians or None if failed
        """
        if not self._is_connected:
            return None
            
        try:
            result = self._get_cached_parameter(Path.CURRENT_JOINT_POSE_RADIANS.value)
            return list(result.value[6:7]) if result else None
        except Exception as e:
            logger.error(f"Error getting motor positions (radians): {e}")
            return None
    
    def getManipulability(self) -> Optional[float]:
        """
        Description:
            Ability to maneuver range 0..1
            0 - critical situation
            1 - safe situation
        
        Returns:
            float: actual manipulability or None if failed
        """
        if not self._is_connected:
            return None
            
        try:
            result = self._get_cached_parameter(Path.MANIPULABILITY_CMD.value)
            return result.value[0] if result else None
        except Exception as e:
            logger.error(f"Error getting manipulability: {e}")
            return None
        
    def getActualTemperature(self) -> Optional[List[float]]:
        """
        Returns:
            list(float): actual motor temperatures or None if failed
        """
        if not self._is_connected:
            return None
            
        try:
            self.__req.setParameter(Path.READSDO_CMD.value, 1).get()
            actualTemperature = []
            for i in range(6):
                result = self._get_cached_parameter(Path.ACTUAL_TEMPERATURE_CMD.value[i])
                actualTemperature.append(result.value[0] if result else 0.0)

            self.__req.setParameter(Path.READSDO_CMD.value, 0).get()
            return actualTemperature
        except Exception as e:
            logger.error(f"Error getting temperature: {e}")
            return None
        
    def __cap_velocity(self, velocities, max_velocity):
        return [min(max(velocity, -max_velocity), max_velocity) for velocity in velocities]

class Conveyer():
    def __init__(self, req: Request):
        self._req = req
        self.status = True
        self._lock = threading.RLock()
    
    def start(self) -> bool:
        if True:
            if not self.status:
                try:
                    self._req.setParameter(Path.CONVEYER_PINS.value[0], 1).get()
                    self.status = True
                    return True
                except Exception as e:
                    logger.error(f"Failed to start conveyer: {e}")
                    return False
            return True
    
    def stop(self) -> bool:
        if True:
            if self.status:
                try:
                    self._req.setParameter(Path.CONVEYER_PINS.value[0], 0).get()
                    self.status = False
                    return True
                except Exception as e:
                    logger.error(f"Failed to stop conveyer: {e}")
                    return False
            return True