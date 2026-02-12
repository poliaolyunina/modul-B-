import state 
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton
import sys

def run():
    app = QApplication(sys.argv)
    window = QMainWindow ()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    run()