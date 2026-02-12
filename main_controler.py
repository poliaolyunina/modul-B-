import tkinter as tk
from tkinter import messagebox
import random
import time
from motion.core import LedLamp
from state import Command
from PyQt5.QtWidgets import QApplication 


 
class RobotStates(): 
    def __init__(self):
        self.Command = RobotStates ()
        self.current_position = [0, 0, 0, 0, 0, 0]  
        self.is_running = False
        self.is_paused = False
        self.is_emergency = False
        
    def execute_command(self, command, AxisState=None, ):
       
        if isinstance(command, str):
            try:
                command = Command(command.upper())
            except ValueError:
                print(f" Неизвестная команда: {command}")
                return False

        print(f" {command.name} = '{command.value}'")
        
        if command == Command.ON:
            return self._enable()
        elif command == Command.OFF:
            return self._shutdown()
        elif command == Command.PAUSE:
            return self._pause()
        elif command == Command.EMERGENCY:
            return self._emergency()
        elif command == Command.WAIT:
            return self._move_to_start()
        elif command == Command.START:
            return self.GETTING_STARTED()
        elif command == Command.EXIT:
            return self._exit()
    
    def enable(self):
        
        if not self.is_running and not self.is_emergency:
            self.is_running = True
            print("робот включен")
            self.current_position = [0.5, 0, 0, 0, 0, 0]
            print(f"Текущая позиция: {self.current_position}")
            return True
    def shutdown(self):
       
        self.is_running = False
        self.is_paused = False
        print("робот выключен ")
        return True
    
    def pause(self):
        if self.is_running and not self.is_emergency:
            self.is_paused = not self.is_paused
            status = "пауза" if self.is_paused else "возвращение на старт "
            self.current_position = [0, 0, 0, 0, 0, 0]
            print(f"{status}")
            return True
    
    def emergency(self):
        self.is_emergency = True
        self.is_running = False
        self.is_paused = False
        self.current_position = [0, 0, 0, 0, 0, 0]
        print("Экстренное торможение")
        return True
    
    def MoveToStart(self):

        if not self.is_emergency:
            print("Возвращение на старт")
            self.current_position = [0.5, 0, 0, 0, 0, 0]
            print(f" {self.current_position}")
            return True

    
    def GettingStarted(self):
        print("Начало работы в ручном режиме")
        self.current_position = [0, 0, 0, 0, 0, 0]
        self.is_emergency = False
        print(f" : {self.current_position}")
        return True
    
    def exit(self):
        print("Выход")
        return True
    

class QPushButton :
    def __init__(self, root):
        self.root = root
        self.root.title()
        self.pool = ["+"]
        self.move_list = []
        moving = False     
        auto_mode = False 

        self.setup_ui()
        
    def setup_ui(self):
        title = tk.Label(self.root)
        print("автоматическое премещение объектов ")
    

    def start_auto_move():
        if not moving:
            moving = True
            auto_mode = True   
        print(" движение по точкам")

def clear_list():
   
        print("Список очищен")

class logoo(): 
    def root(self):
        root = tk.Tk()
        root.geometry("500x700")
        print("Перемещение объектов логами")
    items = []
    moving = False
    auto_mode = False




            
            