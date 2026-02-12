import tkinter as tk
import time 
from main_controler import RobotStates
class logoo(): 
    def root(self):
        root = tk.Tk()
        root.geometry("500x700")
        print("Перемещение объектов логами")
    items = []
    moving = False
    auto_mode = False


