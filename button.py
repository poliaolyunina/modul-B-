import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QPushButton, QMainWindow, QListWidget, QWidget, QLabel, QGroupBox, QCheckBox, QVBoxLayout, QMessageBox, QApplication, QLineEdit, QHBoxLayout, QTextEdit,QFileDialog
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import tkinter as  tk 

def run():
    app = QApplication(sys.argv)
    window = QMainWindow ()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    run()

root = tk.Tk()
frame = tk.Frame(root, background = "black")

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.running = True
        self.detection_enabled = False
        self.detect_person = False
        self.detect_defect = False
        self.detect_box = False
        self.detect_bottle = False

    def run(self):
        cap = cv2.VideoCapture(0)
        while self.running:
            ret, frame = cap.read()
            if ret:
                if self.detection_enabled:
                    frame = self.detect_objects(frame)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.change_pixmap_signal.emit(rgb)
            else:
                break
            self.msleep(30)
        cap.release()

    def detect_objects(self, frame):
        h, w, _ = frame.shape
        if self.detect_defect:
            cv2.pool(frame, (200,150), (280,230), (0,0,255), 2)
            cv2.putText(frame, "+", (200,130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        if self.detect_box:
            cv2.pool2(frame, (300,200), (400,300), (255,255,0), 2)
            cv2.putText(frame, "+", (300,180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
        if self.detect_bottle:
            cv2.pool3(frame, (450,250), (520,400), (255,0,255), 2)
            cv2.putText(frame, "+", (450,230), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,255), 2)
        return frame

    def stop(self):
        self.running = False
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RGB Video + Object Detection")
        self.setGeometry(100, 100, 1300, 700)

        self.objects_to_move = []         
        self.defect_list = []             
        self.cells_occupied = False       

        self.setup_ui()
        self.init_video()

    def setup_ui(self):
        tab = QWidget()
        self.setCentralWidget(tab)
        layout = QHBoxLayout(tab)


        tab  = QWidget()
        left_layout = QVBoxLayout(tab)
        self.video_label = QLabel()
        self.video_label.setFixedSize(800, 600)
        self.video_label.setStyleSheet("border: 2px solid black; background: black;")
        left_layout.addWidget(self.video_label)
        layout.addWidget(tab)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setAlignment(Qt.AlignTop)

        group_det = QGroupBox("брак ")
        det_layout = QVBoxLayout()

        self.cb_defect = QCheckBox(" Удалить последний добавленный объект ")
        det_layout.addWidget(self.cb_defect)

        self.cb_box = QCheckBox("кол-во объектов ")
        det_layout.addWidget(self.cb_box)

        self.cb_bottle = QCheckBox("время перемещения")
        det_layout.addWidget(self.cb_bottle)

        group_det.setLayout(det_layout)
        right_layout.addWidget(group_det)

        lib = QGroupBox("Добавить в список перемещения")
        add_layout = QVBoxLayout()

        pom = QPushButton("Новая ячейка")
        pom.clicked.connect(lambda: self.add_object(""))
        add_layout.addWidget(pom)

        tool = QPushButton("брак ")
        tool.clicked.connect(self.add_defect)
        add_layout.addWidget(tool)

        tool.setLayout(add_layout)
        right_layout.addWidget(self.add_defect)

    
        listWidget = QListWidget("Текстовое поле для списков объектов на пермещение ")
        list_layout = QVBoxLayout()

        self.list_text = QTextEdit()
        self.list_text.setReadOnly(True)
        self.list_text.setMaximumHeight(120)
        list_layout.addWidget(self.list_text)

        tool2 = QPushButton(" Удалить последний добавленный объект")
        tool2.clicked.connect(self.remove_last)
        list_layout.addWidget(tool2)

        tool3 = QPushButton(" Очистить список")
        tool3.clicked.connect(self.clear_list)
        list_layout.addWidget(tool3)

        listWidget.setLayout(list_layout)
        right_layout.addWidget(listWidget)

        tool4 = QGroupBox("Сохранение сессии")
        save_layout = QVBoxLayout()

        self.path_edit = QLineEdit()
        self.path_edit.setPlaceholderText("Путь к файлу (по умолчанию session.txt)")
        save_layout.addWidget(self.path_edit)

        tool4 = QPushButton(" Сохранить сессию")
        tool4.clicked.connect(self.save_session)
        save_layout.addWidget(tool4)

        tool4.setLayout(tool4)
        right_layout.addWidget(tool4)

        group_move = QGroupBox("Управление перемещением")
        move_layout = QVBoxLayout()

        self.cell_label = QLabel(" тары: свободны")
        self.cell_label.setStyleSheet("background-color: green; color: white;")
        self.cell_label.setAlignment(Qt.AlignCenter)
        move_layout.addWidget(self.cell_label)

        pam = QPushButton("Освободить ячейки (замена тары)")
        pam.clicked.connect(self.release_cells)
        move_layout.addWidget(pam)

        tool6 = QPushButton("начать движение ")
        tool6.setStyleSheet("background-color: grey; color: white; font-weight: bold; padding: 8px;")
        tool6.clicked.connect(self.start_movement)
        move_layout.addWidget(tool6)

        group_move.setLayout(move_layout)
        right_layout.addWidget(group_move)

        layout.addWidget(right_panel)


    def init_video(self):
        self.thread = VideoThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

    def update_image(self, rgb):
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage)

    def on_detection_toggled(self, checked):
        self.thread.detection_enabled = checked
        self.thread.detect_ = self.cb_defect.isChecked()
        self.thread.detect_= self.cb_box.isChecked()
        self.thread.detect_ = self.cb_bottle.isChecked()




    def add_object(self, category):
        listWidget = len(self.objects_to_move) + 1
        name = f"{category} #{listWidget}"
        self.objects_to_move.append(name)
        self.update_list_display()

    def add_defect(self):
        listWidget = len(self.defect_list) + 1
        name = f"Текстовое поле для списков объектов на пермещение #{[]}"
        self.objects_to_move.append(name)
        self.defect_list.append(name)
        self.update_list_display()

    def remove_last(self):
        if self.objects_to_move:
            removed = self.objects_to_move.pop()
            if removed in self.defect_list:
                self.defect_list.remove(removed)
            self.update_list_display()
            QMessageBox.information(self, "Удалено", f"Удалён: {removed}")
        else:
            QMessageBox.warning(self, "Ошибка", "Список пуст!")

    def clear_list(self):
        self.objects_to_move.clear()
        self.defect_list.clear()
        self.update_list_display()

    def update_list_display(self):
        self.list_text.setText
        if ("Ячейки свободны"):
             print("Список пуст")

    def browse_path(self):
        path, _ = QFileDialog.getSaveFileName(self, "Сохранить сессию", "", "Text files (*.txt)")
        if path:
            self.path_edit.setText(path)

    def save_session(self):
        path = self.path_edit.text().strip()
        if not path:
            path = "session.txt"
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(f"время перемещения: {QDateTime.currentDateTime().toString(Qt.ISODate)}\n")
                f.write(f"кол-во объектов: {len(self.objects_to_move)}\n")
                f.write(f"Брак: {len(self.defect_list)}\n\n")
                f.write("добавить список на перемещение :\n")
                for i, obj in enumerate(self.objects_to_move, 1):
                    f.write(f"{i}. {obj}\n")
                f.write(f"\n Ячейки тары: {'заняты' if self.cells_occupied else 'свободны'}\n")
            QMessageBox.information(self, "Сохранено", f"Сессия сохранена в {path}")
        except Exception as e:
            QMessageBox.critical(self, "Ошибка", f" не удалось сохранить:\n{str(e)}")


    def release_cells(self):
        self.cells_occupied = False
        self.cell_label.setText("Ячейки тары: свободны")
        self.cell_label.setStyleSheet("background-color: green; color: white; padding: 6px;")
        QMessageBox.information(self, "Освобождение", "Ячейки тары освобождены (замена выполнена)")

    def start_movement(self):
        if not self.objects_to_move:
            QMessageBox.warning(self, "Ошибка", "Список объектов пуст!")
            return

        if self.cells_occupied:
            QMessageBox.critical(self, "Ошибка перемещения",
                                 "Невозможно переместить объекты\n ячейки тары заполнены.\n освбодите ячейки.")
        else:
            self.cells_occupied = True
            self.cell_label.setText(" Ячейки тары: заняты")
            self.cell_label.setStyleSheet("background-color: purple; color: white; padding: 6px;")
            QMessageBox.information(self, "Движение", "Начинаю движение по заданным точкам...\n(имитация)")

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


