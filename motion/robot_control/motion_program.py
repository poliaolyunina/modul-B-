#!/usr/bin/python3

#
#   Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
#   All rights reserved. Copyright (c) 2017 - 2024 VECTIONEER.
#

import itertools
import robot_control
from robot_control.system_defs import FrameTypes


def sendProgramList(req, motorcortex_types, motion_programs, systems_id):
    """Sends programs to the multiple systems

        Args:
            req(motorcortex.Request): reference to a Request instance
            motorcortex_types(motorcortex.MessageTypes): reference to a MessageTypes instance
            motion_programs(list(MotionProgram)): a list of motion programs
            systems_id(list(int)): a list of systems id
    """

    MotionSpec = motorcortex_types.getNamespace("motion_spec_v1")
    motion_program_list = MotionSpec.MotionProgramList()

    number_of_motion_programs = len(motion_programs)
    number_of_systems = len(systems_id)
    if number_of_motion_programs != number_of_systems:
        raise "Number of motion programs must be equal to the number of subsystems. "

    for i in range(number_of_motion_programs):
        motion_program_list.motionprogramlist.append(motion_programs[i].generateMessage(system_id=systems_id[i]))

    return req.send(motorcortex_types.encode(motion_program_list))


class Waypoint(object):
    """Class represents a waypoint of the motion path

        Args:
            pose(list(double)): pose in Cartesian or joint space
            smoothing_factor(double): waypoint smoothing factor in the range [0..1]
            next_segment_velocity_factor(double) segment velocity factor in the range [0..1]

    """

    def __init__(self, pose, smoothing_factor=0.1, next_segment_velocity_factor=1.0):
        self.pose = pose
        self.smoothing_factor = smoothing_factor
        self.next_segment_velocity_factor = next_segment_velocity_factor


class PoseTransformer(object):
    """Convert Cartesian tooltip to joint angles and the other way round

        Args:
            req(motorcortex.Request): reference to a Request instance
            motorcortex_types(motorcortex.MessageTypes): reference to a MessageTypes instance
    """

    def __init__(self, req, motorcortex_types):
        self.__MotionSpec = motorcortex_types.getNamespace("motion_spec")
        self.__MotionSpec_v1 = motorcortex_types.getNamespace("motion_spec_v1")
        if not self.__MotionSpec or not self.__MotionSpec_v1:
            robot_control.init(motorcortex_types)
            self.__MotionSpec = motorcortex_types.getNamespace("motion_spec")
            self.__MotionSpec_v1 = motorcortex_types.getNamespace("motion_spec_v1")

        self.__motorcortex_types = motorcortex_types
        self.__req = req
        self.__request_id = itertools.count()

    def calcCartToJointPose(self, cart_coord=None,
                            ref_joint_coord_rad=None,
                            system_id=None):
        """Converts Cartesian tooltip pose to joint coordinates

            Args:
                cart_coord(list(double)): Cartesian coordinates of the tooltip
                ref_joint_coord_rad(list(double)): actual joint coordinates, rad

            Returns:
                motion_spec.CartToJoint: Joint angles, which correspond to Cartesian coordinates,
                with respect to actual joint positions.

        """

        if ref_joint_coord_rad is None:
            ref_joint_coord_rad = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if cart_coord is None:
            cart_coord = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        request_id = next(self.__request_id)
        if system_id:
            cart_to_joint_list_req = self.__MotionSpec_v1.CartToJointList()
            cart_to_joint_list_req.system_id = system_id
        else:
            cart_to_joint_list_req = self.__MotionSpec.CartToJointList()

        cart_to_joint_req = cart_to_joint_list_req.carttojointlist.add()
        cart_to_joint_req.cartpose.coordinates.extend(cart_coord)
        cart_to_joint_req.cartpose.id = request_id
        cart_to_joint_req.jointpose.coordinates.extend(ref_joint_coord_rad)
        cart_to_joint_req.jointpose.id = request_id
        cart_to_joint_req.carttwist.coordinates.extend([])
        cart_to_joint_req.jointtwist.coordinates.extend([])
        cart_to_joint_req.frame_type = FrameTypes.TOOLTIP.value

        return self.__req.send(self.__motorcortex_types.encode(cart_to_joint_list_req)).get()

    def calcJointToCartPose(self, joint_coord_rad=None,
                            cart_coord=None, system_id=None):
        """Converts joint coordinates to Cartesian tooltip pose.

            Args:
                joint_coord_rad(list(double)): joint coordinates, rad
                cart_coord(list(double)): actual Cartesian tooltip pose

            Returns:
                motion_spec.JointToCart: Cartesian tooltip pose, which correspond to joint angles,
                with respect to the actual pose.

        """

        if joint_coord_rad is None:
            joint_coord_rad = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if cart_coord is None:
            cart_coord = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        request_id = next(self.__request_id)
        if system_id:
            joint_to_cart_list_req = self.__MotionSpec_v1.JointToCartList()
            joint_to_cart_list_req.system_id = system_id
        else:
            joint_to_cart_list_req = self.__MotionSpec.JointToCartList()

        joint_to_cart_req = joint_to_cart_list_req.jointtocartlist.add()
        joint_to_cart_req.cartpose.coordinates.extend(cart_coord)
        joint_to_cart_req.cartpose.id = request_id
        joint_to_cart_req.jointpose.coordinates.extend(joint_coord_rad)
        joint_to_cart_req.jointpose.id = request_id
        joint_to_cart_req.carttwist.coordinates.extend([])
        joint_to_cart_req.jointtwist.coordinates.extend([])

        return self.__req.send(self.__motorcortex_types.encode(joint_to_cart_list_req)).get()


class MotionProgram(object):
    """Class represents a motion program of the manipulator

        Args:
            req(motorcortex.Request): reference to a Request instance
            motorcortex_types(motorcortex.MessageTypes): reference to a MessageTypes instance
    """

    def __init__(self, req, motorcortex_types, use_system_id=False):
        self.__Motorcortex = motorcortex_types.getNamespace("motorcortex")
        self.__MotionSpec = motorcortex_types.getNamespace("motion_spec")
        self.__MotionSpec_v1 = motorcortex_types.getNamespace("motion_spec_v1")
        if not self.__MotionSpec or not self.__MotionSpec_v1:
            robot_control.init(motorcortex_types)
            self.__MotionSpec = motorcortex_types.getNamespace("motion_spec")
            self.__MotionSpec_v1 = motorcortex_types.getNamespace("motion_spec_v1")

        self.__motorcortex_types = motorcortex_types
        self.__req = req
        self.__use_system_id = use_system_id
        if use_system_id:
            self.__motion_program = self.__MotionSpec_v1.MotionProgram()
        else:
            self.__motion_program = self.__MotionSpec.MotionProgram()

        self.__cmd_counter = 1
        self.__id = 1

    def clear(self):
        """Clears all commands in the program"""
        if self.__use_system_id:
            self.__motion_program = self.__MotionSpec_v1.MotionProgram()
        else:
            self.__motion_program = self.__MotionSpec.MotionProgram()
        self.__cmd_counter = 1

    def addCommand(self, command, type):
        """Adds a command to the program

            Args:
                command(motion_spec.MotionCommand): motion command from motionSL.proto
                type(motion_spec.MOTIONTYPE): type of the motion command
        """
        motion_cmd = self.__motion_program.commandlist.add()
        motion_cmd.id = self.__cmd_counter
        motion_cmd.commandtype = type
        motion_cmd.commandarguments = command.SerializeToString()
        self.__cmd_counter = self.__cmd_counter + 1

    def addMoveC(self, waypoint_list, angle, velocity=0.1, acceleration=0.2,
                 rotational_velocity=3.18, rotational_acceleration=6.37,
                 ref_joint_coord_rad=None):
        """Adds a MoveC(circular move) command to the program

            Args:
                waypoint_list(list(WayPoint)): a list of waypoints
                angle(double): rotation angle, rad
                velocity(double): maximum velocity, m/sec
                acceleration(double): maximum acceleration, m/sec^2
                rotational_velocity(double): maximum joint velocity, rad/sec
                rotational_acceleration(double): maximum joint acceleration, rad/sec^2
                ref_joint_coord_rad: reference joint coordinates for the first waypoint

        """

        if ref_joint_coord_rad is None:
            ref_joint_coord_rad = []
        move_c = self.__MotionSpec.MoveC()
        move_c.constraint.type = self.__MotionSpec.VELANDACC
        move_c.constraint.velacc_values.vMax = velocity
        move_c.constraint.velacc_values.aMax = acceleration
        move_c.angle = angle
        move_c.constraint.velacc_values.omegaMax = rotational_velocity
        move_c.constraint.velacc_values.alfaMax = rotational_acceleration
        move_c.referenceJoint.coordinates.extend(ref_joint_coord_rad)

        for waypoint in waypoint_list:
            ms_waypoint_ref = move_c.waypoints.add()
            ms_waypoint_ref.constraint.type = self.__MotionSpec.POSITION
            ms_waypoint_ref.segmentVelocity = waypoint.next_segment_velocity_factor
            ms_waypoint_ref.constraint.factor = waypoint.smoothing_factor
            ms_waypoint_ref.pose.coordinates.extend(waypoint.pose)

        self.addCommand(move_c, self.__MotionSpec.ARC)

    def createMoveC(self, waypoint_list, angle, velocity=0.1, acceleration=0.2,
                    rotational_velocity=3.18, rotational_acceleration=6.37,
                    ref_joint_coord_rad=None):
        """Creates a MoveC(circular move) command to the program

        Args:
            waypoint_list(list(WayPoint)): a list of waypoints
            angle(double): rotation angle, rad
            velocity(double): maximum velocity, m/sec
            acceleration(double): maximum acceleration, m/sec^2
            rotational_velocity(double): maximum joint velocity, rad/sec
            rotational_acceleration(double): maximum joint acceleration, rad/sec^2
            ref_joint_coord_rad: reference joint coordinates for the first waypoint

        Returns:
            motion_spec.MoveC: returns MoveC command
        """

        if ref_joint_coord_rad is None:
            ref_joint_coord_rad = []
        move_c = self.__MotionSpec.MoveC()
        move_c.constraint.type = self.__MotionSpec.VELANDACC
        move_c.constraint.velacc_values.vMax = velocity
        move_c.constraint.velacc_values.aMax = acceleration
        move_c.angle = angle
        move_c.constraint.velacc_values.omegaMax = rotational_velocity
        move_c.constraint.velacc_values.alfaMax = rotational_acceleration
        move_c.referenceJoint.coordinates.extend(ref_joint_coord_rad)

        for waypoint in waypoint_list:
            ms_waypoint_ref = move_c.waypoints.add()
            ms_waypoint_ref.constraint.type = self.__MotionSpec.POSITION
            ms_waypoint_ref.segmentVelocity = waypoint.next_segment_velocity_factor
            ms_waypoint_ref.constraint.factor = waypoint.smoothing_factor
            ms_waypoint_ref.pose.coordinates.extend(waypoint.pose)

        return move_c

    def addMoveL(self, waypoint_list, velocity=0.1, acceleration=0.2,
                 rotational_velocity=3.18, rotational_acceleration=6.37,
                 ref_joint_coord_rad=None):
        """Adds a MoveL(Linear move) command to the program

            Args:
                waypoint_list(list(WayPoint)): a list of waypoints
                velocity(double): maximum velocity, m/sec
                acceleration(double): maximum acceleration, m/sec^2
                rotational_velocity(double): maximum joint velocity, rad/sec
                rotational_acceleration(double): maximum joint acceleration, rad/sec^2
                ref_joint_coord_rad: reference joint coordinates for the first waypoint

        """

        if ref_joint_coord_rad is None:
            ref_joint_coord_rad = []
        move_l = self.__MotionSpec.MoveL()
        move_l.constraint.type = self.__MotionSpec.VELANDACC
        move_l.constraint.velacc_values.vMax = velocity
        move_l.constraint.velacc_values.aMax = acceleration
        move_l.constraint.velacc_values.omegaMax = rotational_velocity
        move_l.constraint.velacc_values.alfaMax = rotational_acceleration
        move_l.referenceJoint.coordinates.extend(ref_joint_coord_rad)

        for waypoint in waypoint_list:
            ms_waypoint_ref = move_l.waypoints.add()
            ms_waypoint_ref.constraint.type = self.__MotionSpec.POSITION
            ms_waypoint_ref.segmentVelocity = waypoint.next_segment_velocity_factor
            ms_waypoint_ref.constraint.factor = waypoint.smoothing_factor
            ms_waypoint_ref.pose.coordinates.extend(waypoint.pose)

        self.addCommand(move_l, self.__MotionSpec.CARTMOTION)

    def createMoveL(self, waypoint_list, velocity=0.1, acceleration=0.2,
                    rotational_velocity=3.18, rotational_acceleration=6.37,
                    ref_joint_coord_rad=None):
        """Adds a MoveL(Linear move) command to the program

            Args:
                waypoint_list(list(WayPoint)): a list of waypoints
                velocity(double): maximum velocity, m/sec
                acceleration(double): maximum acceleration, m/sec^2
                rotational_velocity(double): maximum joint velocity, rad/sec
                rotational_acceleration(double): maximum joint acceleration, rad/sec^2
                ref_joint_coord_rad: reference joint coordinates for the first waypoint

            Returns:
                motion_spec.MoveL: returns MoveL command
        """

        if ref_joint_coord_rad is None:
            ref_joint_coord_rad = []
        move_l = self.__MotionSpec.MoveL()
        move_l.constraint.type = self.__MotionSpec.VELANDACC
        move_l.constraint.velacc_values.vMax = velocity
        move_l.constraint.velacc_values.aMax = acceleration
        move_l.constraint.velacc_values.omegaMax = rotational_velocity
        move_l.constraint.velacc_values.alfaMax = rotational_acceleration
        move_l.referenceJoint.coordinates.extend(ref_joint_coord_rad)

        for waypoint in waypoint_list:
            ms_waypoint_ref = move_l.waypoints.add()
            ms_waypoint_ref.constraint.type = self.__MotionSpec.POSITION
            ms_waypoint_ref.segmentVelocity = waypoint.next_segment_velocity_factor
            ms_waypoint_ref.constraint.factor = waypoint.smoothing_factor
            ms_waypoint_ref.pose.coordinates.extend(waypoint.pose)

        return move_l

    def addComposedCartMove(self, cart_move_list):
        composed_cart = self.__MotionSpec.ComposedCartMove()
        cart = self.__MotionSpec.CartMove()
        for cart_move in cart_move_list:
            if cart_move.DESCRIPTOR.full_name == 'motion_spec.MoveL':
                cart.movel.CopyFrom(cart_move)
            elif cart_move.DESCRIPTOR.full_name == 'motion_spec.MoveC':
                cart.movec.CopyFrom(cart_move)

            composed_cart.cart_move.append(cart)

        self.addCommand(composed_cart, self.__MotionSpec.COMPOSED_CART)

    def addMoveJ(self, waypoint_list, rotational_velocity=3.18, rotational_acceleration=6.37):
        """Adds MoveJ(Joint move) command to the program

            Args:
                waypoint_list(list(WayPoint)): a list of waypoints
                rotational_velocity(double): maximum joint velocity, rad/sec
                rotational_acceleration(double): maximum joint acceleration, rad/sec^2

        """

        move_j = self.__MotionSpec.MoveJ()
        move_j.constraint.type = self.__MotionSpec.VELANDACC
        move_j.constraint.velacc_values.omegaMax = rotational_velocity
        move_j.constraint.velacc_values.alfaMax = rotational_acceleration
        move_j.constraint.velacc_values.vMax = 0
        move_j.constraint.velacc_values.aMax = 0

        for waypoint in waypoint_list:
            ms_waypoint_ref = move_j.waypoints.add()
            ms_waypoint_ref.constraint.type = self.__MotionSpec.TIME
            ms_waypoint_ref.segmentVelocity = waypoint.next_segment_velocity_factor
            ms_waypoint_ref.constraint.factor = waypoint.smoothing_factor
            ms_waypoint_ref.pose.coordinates.extend(waypoint.pose)

        self.addCommand(move_j, self.__MotionSpec.JOINTMOTION)

    def addWait(self, timeout_s, path=None, value=1):
        """Adds Wait command to the program

            Args:
                timeout_s(double): time to wait in seconds
                path(string): path to the parameter that will be compared to value
                value(double): value that the parameter is compared to
        """

        wait_cmd = self.__MotionSpec.Wait()
        wait_cmd.timeout = timeout_s
        if path is not None:
            wait_cmd.path = path
            wait_cmd.value = value
        self.addCommand(wait_cmd, self.__MotionSpec.WAIT)

    def addSetDoubleParameter(self, path=None, value=0.0):
        set_parameter_cmd = self.__MotionSpec.SetDouble()
        if path is not None:
            set_parameter_cmd.path = path
            set_parameter_cmd.value = value
        self.addCommand(set_parameter_cmd, self.__MotionSpec.SETDOUBLE)
        
    def addSetParameter(self, path=None, value=0):
        set_parameter_cmd = self.__MotionSpec.Set()
        if path is not None:
            set_parameter_cmd.path = path
            set_parameter_cmd.value = value
        self.addCommand(set_parameter_cmd, self.__MotionSpec.SET)

    def addLinearTrackMove(self, position, velocity=0.1, acceleration=1.0):
        move_l = self.__MotionSpec.MoveL()
        move_l.constraint.type = self.__MotionSpec.VELANDACC
        move_l.referenceJoint.coordinates.extend([])
        self.addCommand(move_l, self.__MotionSpec.CARTMOTION)
        self.addSetDoubleParameter("root/MachineControl/autoModeGenerator/maxVel", velocity)
        self.addSetDoubleParameter("root/MachineControl/autoModeGenerator/maxAcc", acceleration)
        self.addSetDoubleParameter("root/MachineControl/autoModeGenerator/target", position)
        self.addWait(timeout_s=-1.0, path="root/MachineControl/autoModeGenerator/targetStatus/allJointsWithinTolerance", value=1)


    # def addLinearTrackMove(self, position, velocity=0.05, acceleration=0.1):
    #     self.addSetDoubleParameter("root/MachineControl/autoModeGenerator/maxVel", velocity)
    #     self.addSetDoubleParameter("root/MachineControl/autoModeGenerator/maxAcc", acceleration)
    #     self.addSetDoubleParameter("root/MachineControl/autoModeGenerator/target", position)
    #     self.addWait(timeout_s=-1.0, path="root/MachineControl/autoModeGenerator/targetStatus/allJointsWithinTolerance", value=1)
        
    def generateMessage(self, program_name='Undefined', system_id=0):
        self.__motion_program.name = program_name
        self.__motion_program.id = self.__id
        if self.__use_system_id:
            self.__motion_program.system_id = system_id

        self.__id = self.__id + 1
        return self.__motion_program

    def send(self, program_name='Undefined', system_id=0):
        """Sends program to the robot

            Args:
                program_name(str): program name

        """

        return self.__req.send(self.__motorcortex_types.encode(self.generateMessage(program_name, system_id)))
