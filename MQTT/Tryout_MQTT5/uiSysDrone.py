import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from uiSystemDrone import Ui_MainWindow

class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Initialize the UI class
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())