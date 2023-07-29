import sys
import cv2
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QSlider

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 创建一个标签用于显示视频流
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.setCentralWidget(self.image_label)

        # 创建六个滑动条，用于调整HSV值
        self.h_min_slider = QSlider(Qt.Horizontal)
        self.h_min_slider.setMinimum(0)
        self.h_min_slider.setMaximum(255)
        self.h_min_slider.setValue(0)
        self.h_min_slider.valueChanged.connect(self.update_mask)
        # layout.addWidget(self.h_min_slider)

        self.s_min_slider = QSlider(Qt.Horizontal)
        self.s_min_slider.setMinimum(0)
        self.s_min_slider.setMaximum(255)
        self.s_min_slider.setValue(0)
        self.s_min_slider.valueChanged.connect(self.update_mask)
        # layout.addWidget(self.s_min_slider)

        self.v_min_slider = QSlider(Qt.Horizontal)
        self.v_min_slider.setMinimum(0)
        self.v_min_slider.setMaximum(255)
        self.v_min_slider.setValue(0)
        self.v_min_slider.valueChanged.connect(self.update_mask)
        # layout.addWidget(self.v_min_slider)

        self.h_max_slider = QSlider(Qt.Horizontal)
        self.h_max_slider.setMinimum(0)
        self.h_max_slider.setMaximum(255)
        self.h_max_slider.setValue(255)
        self.h_max_slider.valueChanged.connect(self.update_mask)
        # layout.addWidget(self.h_max_slider)

        self.s_max_slider = QSlider(Qt.Horizontal)
        self.s_max_slider.setMinimum(0)
        self.s_max_slider.setMaximum(255)
        self.s_max_slider.setValue(255)
        self.s_max_slider.valueChanged.connect(self.update_mask)
        # layout.addWidget(self.s_max_slider)

        self.v_max_slider = QSlider(Qt.Horizontal)
        self.v_max_slider.setMinimum(0)
        self.v_max_slider.setMaximum(255)
        self.v_max_slider.setValue(255)
        self.v_max_slider.valueChanged.connect(self.update_mask)
        # layout.addWidget(self.v_max_slider)

        # 创建一个垂直布局，用于放置滑动条和视频流标签
        layout = QVBoxLayout()
        layout.addWidget(self.h_min_slider)
        layout.addWidget(self.s_min_slider)
        layout.addWidget(self.v_min_slider)
        layout.addWidget(self.h_max_slider)
        layout.addWidget(self.s_max_slider)
        layout.addWidget(self.v_max_slider)
        layout.addWidget(self.image_label)

        # 创建一个窗口部件，用于放置垂直布局
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        # 创建一个定时器，用于定期更新视频流
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(50)

        # 打开摄像头
        self.cap = cv2.VideoCapture(0)

    def update_frame(self):
        # 从摄像头读取一帧
        ret, frame = self.cap.read()

        # 将帧转换为Qt图像
        if ret:
            image = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(image)
            self.image_label.setPixmap(pixmap)

    def update_mask(self):
        # 从摄像头读取一帧
        ret, frame = self.cap.read()

        # 将帧转换为HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 获取滑动条的值
        h_min = self.h_min_slider.value()
        s_min = self.s_min_slider.value()
        v_min = self.v_min_slider.value()
        h_max = self.h_max_slider.value()
        s_max = self.s_max_slider.value()
        v_max = self.v_max_slider.value()

        # 创建两个掩码，用于过滤颜色
        lower = (h_min, s_min, v_min)
        upper = (h_max, s_max, v_max)
        mask = cv2.inRange(hsv, lower, upper)
        inv_mask = cv2.bitwise_not(mask)

        # 将掩码应用于原始图像
        result = cv2.bitwise_and(frame, frame, mask=inv_mask)
        
        # 将掩码转换为三维数组
        mask_3d = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # 将原始图像和掩码连接起来
        combined_image = np.concatenate((frame, mask_3d), axis=1)
        # 将连接后的图像转换为Qt图像
        combined_image = cv2.cvtColor(combined_image, cv2.COLOR_BGR2RGB)
        combined_image = QImage(combined_image, combined_image.shape[1], combined_image.shape[0], QImage.Format_RGB888)
        combined_pixmap = QPixmap.fromImage(combined_image)

        # 将掩码转换为Qt图像
        mask_image = QImage(mask, mask.shape[1], mask.shape[0], QImage.Format_Grayscale8)
        mask_pixmap = QPixmap.fromImage(mask_image)

        # 将帧和掩码组合成一个图像
        combined_image = np.concatenate((frame, mask), axis=1)
        combined_image = cv2.cvtColor(combined_image, cv2.COLOR_BGR2RGB)

        # 将组合图像转换为Qt图像
        combined_image = QImage(combined_image, combined_image.shape[1], combined_image.shape[0], QImage.Format_RGB888).rgbSwapped()
        combined_pixmap = QPixmap.fromImage(combined_image)

        # 更新标签
        self.image_label.setPixmap(combined_pixmap)
        self.mask_label.setPixmap(mask_pixmap)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())