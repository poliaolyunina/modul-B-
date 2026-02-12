#!/usr/bin/python3

#
#   Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
#   All rights reserved. Copyright (c) 2024 VECTIONEER.
#

from .robot_command import *

"""Checks if all the values in the list are equal to the objective value.

    Args:
        list_of_values(list(any)): list of the values to compare
        objective_value(any): an objective value

    Returns:
        bool: True if operation all is equal, False if not

"""


def isEqual(list_of_values, objective_value):
    return all(objective_value == value for value in list_of_values)


class MultiRobotCommand(object):
    """Class represents a state machine of the multiple robot arms.

        Args:
            req(motorcortex.Request): reference to a Request instance
            motorcortex_types(motorcortex.MessageTypes): reference to a MessageTypes instance
            systems(list(int)): a list of systems id

    """

    def __init__(self, req, motorcortex_types, systems_id=None):
        self.__Motorcortex = motorcortex_types.getNamespace("motorcortex")
        self.__motorcortex_types = motorcortex_types
        self.__req = req
        self.__kinematics_update_counter = 0
        self.__robot_command_list = {}
        if not systems_id:
            self.__systems_id = [None]
            self.__robot_command_list.update({None: RobotCommand(req, motorcortex_types)})
        else:
            self.__systems_id = systems_id
            for system_id in self.__systems_id:
                self.__robot_command_list.update({system_id: RobotCommand(req, motorcortex_types, system_id)})

    def __addSystem(self, name, system):
        if not system:
            return name
        return f"{name}{system:02d}"

    def play(self, systems_id=None, wait_time=1.0):
        """Plays the program.

            Args:
                wait_time(double): short delay after which actual state of the interpreter is requested
                systems_id(list(int)): an optional list of the systems id, a subset of the object defined list

            Returns:
                InterpreterStates: actual state of the program interpreter

        """

        if not systems_id:
            systems_id = self.__systems_id

        paramList = []
        for system_id in systems_id:
            paramList.append({"path": f'root/{self.__addSystem("MotionInterpreter", system_id)}/interpreterCommand',
                              "value": InterpreterEvents.PLAY_PROGRAM_E.value})

        self.__req.setParameterList(paramList)
        time.sleep(wait_time)

        return self.getState(systems_id)

    def pause(self, systems_id=None, wait_time=1.0):
        """Plays the program.

            Args:
                wait_time(double): short delay after which actual state of the interpreter is requested
                systems_id(list(int)): an optional list of the systems id, a subset of the object defined list

            Returns:
                InterpreterStates: actual state of the program interpreter

        """

        if not systems_id:
            systems_id = self.__systems_id

        param_list = []
        for system_id in systems_id:
            param_list.append({"path": f'root/{self.__addSystem("MotionInterpreter", system_id)}/interpreterCommand',
                               "value": InterpreterEvents.PAUSE_PROGRAM_E.value})

        self.__req.setParameterList(param_list)
        time.sleep(wait_time)

        return self.getState(systems_id)

    def getState(self, systems_id=None):
        """
            systems_id(list(int)): an optional list of the systems id, a subset of the object defined list

            Returns:
                InterpreterStates: actual state of the interpreter

        """

        if not systems_id:
            systems_id = self.__systems_id

        param_list = []
        for system_id in systems_id:
            param_list.append(f'root/{self.__addSystem("MotionInterpreter", system_id)}/actualStateOut')

        params = self.__req.getParameterList(param_list).get()

        result = []
        for param in params.params:
            result.append(param.value[0])

        return result

    def engage(self):
        """Switch robot to Engage state.

            Returns:
                bool: True if operation is completed, False if failed

        """
        return self.__robot_command_list[self.__systems_id[0]].engage()

    def stop(self, systems_id=None, wait_time=1.0):
        """Stop the program.

            Args:
                systems_id(list(int)): an optional list of the systems id, a subset of the object defined list
                wait_time(double): short delay after which actual state of the interpreter is requested

            Returns:
                InterpreterStates: actual state of the program interpreter

        """

        if not systems_id:
            systems_id = self.__systems_id

        param_list = []
        for system_id in systems_id:
            param_list.append({"path": f'root/{self.__addSystem("MotionInterpreter", system_id)}/interpreterCommand',
                               "value": InterpreterEvents.STOP_PROGRAM_E.value})

        self.__req.setParameterList(param_list).get()

        time.sleep(wait_time)
        return self.getState(systems_id)

    def reset(self, systems_id=None, wait_time=1.0):
        """Stop the program and clear the interpreter buffer.

            Args:
                systems_id(list(int)): an optional list of the systems id, a subset of the object defined list
                wait_time(double): short delay after which actual state of the interpreter is requested

            Returns:
                InterpreterStates: actual state of the program interpreter

        """

        if not systems_id:
            systems_id = self.__systems_id

        param_list = []
        for system_id in systems_id:
            param_list.append({"path": f'root/{self.__addSystem("MotionInterpreter", system_id)}/interpreterCommand',
                               "value": InterpreterEvents.RESET_INTERPRETER_E.value})

        self.__req.setParameterList(param_list).get()

        time.sleep(wait_time)
        return self.getState(systems_id)

    def moveToStart(self, timeout_s, systems_id=None):
        """Move arm to the start of the program.

            Args:
                timeout_s(double): timeout in seconds
                systems_id(list(int)): an optional list of the systems id, a subset of the object defined list

            Returns:
                bool: True if operation is completed, False if failed

        """
        if not systems_id:
            systems_id = self.__systems_id

        for system_id in systems_id:
            robot_system = self.__robot_command_list[system_id]
            if robot_system.play() is InterpreterStates.MOTION_NOT_ALLOWED_S.value:
                print('Robot is not at a start position, moving to the start')
                if robot_system.moveToStart(timeout_s):
                    print('Robot is at the start position')
                    robot_system.pause()
                else:
                    raise Exception('Failed to move to the start position')

        return self.getState()

    def system(self, system):
        """Move arm to the start of the program.

            Args:
                system(int): id of the system

            Returns:
                RobotCommand: returns a robot command instance for specified system id

        """
        return self.__robot_command_list[system]
