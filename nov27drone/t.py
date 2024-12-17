# from PyQt6.QtWidgets import (
#     QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout,
#     QLabel, QPushButton, QLineEdit, QHBoxLayout
# )
# from PyQt6.QtCore import pyqtSignal, pyqtSlot, QObject, QThread, Qt
# layout = QVBoxLayout() # creates a layout 
# label = QLabel("Live Feed") # this creates thhe label for the feed 
# label.setAlignment(Qt.AlignmentFlag.AlignCenter)
# layout.addWidget(label)

# created a layout but did not add to any parent widgets like q widget or qmainwindow .
# layout by themselves do no display anything , they need to be set on a widget . 



from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt
import sys 
# Main application code
if __name__ == "__main__":
    pyqtapp=QApplication(sys.argv)

    # creating a  windowusing q widget 
    window=QWidget()
    window.setWindowTitle("test")
    window.setGeometry(100,100,600,400)



    # now creating  alayout 
    layout=QVBoxLayout()
    label=QLabel("abhay")
    label.setAlignment(Qt.AlignmentFlag.AlignCenter)

    # adding label to layout
    layout.addWidget(label)
    window.setLayout(layout)
    window.show()
    pyqtapp.exec()