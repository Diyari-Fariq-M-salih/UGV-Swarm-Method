from PyQt5.QtWidgets import QApplication, QMainWindow

class MainWindow:
    def __init__(self):
        self.app = QApplication([])
        self.window = QMainWindow()
        self.window.setWindowTitle("UGV Formation Simulator")

    def run(self):
        self.window.show()
        self.app.exec_()
