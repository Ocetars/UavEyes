import sys
import cv2
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QSlider, QVBoxLayout, QWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 创建一个标签用于显示视频流
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.setCentralWidget(self.image_label)

        # 创建一些滑块用于调整参数
        self.slider1 = QSlider(Qt.Horizontal)
        self.slider1.setMinimum(0)
        self.slider1.setMaximum(255)
        self.slider1.setValue(0)
        self.slider1.setTickPosition(QSlider.TicksBelow)
        self.slider1.setTickInterval(10)

        self.slider2 = QSlider(Qt.Horizontal)
        self.slider2.setMinimum(0)
        self.slider2.setMaximum(255)
        self.slider2.setValue(0)
        self.slider2.setTickPosition(QSlider.TicksBelow)
        self.slider2.setTickInterval(10)

        # 创建一个垂直布局并将滑块添加到其中
        vbox = QVBoxLayout()
        vbox.addWidget(self.slider1)
        vbox.addWidget(self.slider2)

        # 创建一个小部件并将其设置为主窗口的右侧部分
        widget = QWidget()
        widget.setLayout(vbox)
        self.setCentralWidget(widget)

        # 创建一个定时器用于捕获视频流并更新标签
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(50)

        # 打开摄像头并开始捕获视频流
        self.cap = cv2.VideoCapture(0)

    def update_frame(self):
        if self.image_label is None:
            return

        # 从摄像头中读取一帧
        ret, frame = self.cap.read()

        # 将帧转换为Qt图像
        if ret:
            image = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(image)
            self.image_label.setPixmap(pixmap)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
