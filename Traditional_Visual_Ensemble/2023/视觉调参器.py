import cv2
import numpy as np
import sys
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QSlider,
    QVBoxLayout,
    QWidget,
)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        # 设置窗口标题和大小
        self.setWindowTitle("视觉调参器")
        self.setGeometry(150, 150, 850, 650)

        # 创建水平布局
        layout = QHBoxLayout()

        # 创建左侧垂直布局
        left_layout = QVBoxLayout()

        # 创建标签用于显示原始图像
        # self.image_label = QLabel(self)
        # self.image_label.setAlignment(Qt.AlignCenter)
        # left_layout.addWidget(self.image_label)

        # 创建右侧垂直布局
        right_layout = QVBoxLayout()

        # 创建标签用于显示mask图像
        self.mask_label = QLabel(self)
        self.mask_label.setAlignment(Qt.AlignCenter)
        # self.mask_label.setFixedSize(1280*2, 720)
        right_layout.addWidget(self.mask_label)

        # 创建滑动条用于调整颜色阈值
        self.h_min_label = QLabel("H_min: 0", self)
        self.h_min_slider = QSlider(Qt.Horizontal)
        self.h_min_slider.setMinimum(0)
        self.h_min_slider.setMaximum(255)
        self.h_min_slider.setValue(0)
        self.h_min_slider.valueChanged.connect(self.update_mask)
        right_layout.addWidget(self.h_min_label)
        right_layout.addWidget(self.h_min_slider)

        self.s_min_label = QLabel("S_min: 0", self)
        self.s_min_slider = QSlider(Qt.Horizontal)
        self.s_min_slider.setMinimum(0)
        self.s_min_slider.setMaximum(255)
        self.s_min_slider.setValue(0)
        self.s_min_slider.valueChanged.connect(self.update_mask)
        right_layout.addWidget(self.s_min_label)
        right_layout.addWidget(self.s_min_slider)

        self.v_min_label = QLabel("V_min: 0", self)
        self.v_min_slider = QSlider(Qt.Horizontal)
        self.v_min_slider.setMinimum(0)
        self.v_min_slider.setMaximum(255)
        self.v_min_slider.setValue(0)
        self.v_min_slider.valueChanged.connect(self.update_mask)
        right_layout.addWidget(self.v_min_label)
        right_layout.addWidget(self.v_min_slider)

        self.h_max_label = QLabel("H_max: 255", self)
        self.h_max_slider = QSlider(Qt.Horizontal)
        self.h_max_slider.setMinimum(0)
        self.h_max_slider.setMaximum(255)
        self.h_max_slider.setValue(255)
        self.h_max_slider.valueChanged.connect(self.update_mask)
        right_layout.addWidget(self.h_max_label)
        right_layout.addWidget(self.h_max_slider)

        self.s_max_label = QLabel("S_max: 255", self)
        self.s_max_slider = QSlider(Qt.Horizontal)
        self.s_max_slider.setMinimum(0)
        self.s_max_slider.setMaximum(255)
        self.s_max_slider.setValue(255)
        self.s_max_slider.valueChanged.connect(self.update_mask)
        right_layout.addWidget(self.s_max_label)
        right_layout.addWidget(self.s_max_slider)

        self.v_max_label = QLabel("V_max: 255", self)
        self.v_max_slider = QSlider(Qt.Horizontal)
        self.v_max_slider.setMinimum(0)
        self.v_max_slider.setMaximum(255)
        self.v_max_slider.setValue(255)
        self.v_max_slider.valueChanged.connect(self.update_mask)
        right_layout.addWidget(self.v_max_label)
        right_layout.addWidget(self.v_max_slider)

        # 创建标签用于显示HSV阈值范围
        self.hsv_range_label_low = QLabel("HSV阈值下限: H(0), S(0), V(0)", self)
        right_layout.addWidget(self.hsv_range_label_low)
        self.hsv_range_label_high = QLabel("HSV阈值上限: H(255), S(255), V(255)", self)
        right_layout.addWidget(self.hsv_range_label_high)
        

        # 将左侧和右侧布局添加到水平布局中
        layout.addLayout(left_layout)
        layout.addLayout(right_layout)

        # 设置窗口布局
        self.setLayout(layout)

        # 创建摄像头对象
        self.cap = cv2.VideoCapture(2)

        # 设置摄像头分辨率为640x480
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 获取摄像头分辨率
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # 创建定时器用于定时更新摄像头画面
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_mask)
        self.timer.start(25)

        # 显示窗口
        self.show()

    def update_mask(self):
        # 获取滑动条的值
        h_min = self.h_min_slider.value()
        s_min = self.s_min_slider.value()
        v_min = self.v_min_slider.value()
        h_max = self.h_max_slider.value()
        s_max = self.s_max_slider.value()
        v_max = self.v_max_slider.value()

        # 更新HSV阈值范围标签
        self.hsv_range_label_low.setText(
            f"HSV阈值下限: H({h_min}), S({s_min}), V({v_min})"
        )
        self.hsv_range_label_high.setText(
            f"HSV阈值上限: H({h_max}), S({s_max}), V({v_max})"
        )

        # 更新滑动条标签的文本
        self.h_min_label.setText(f"H_min: {h_min}")
        self.s_min_label.setText(f"S_min: {s_min}")
        self.v_min_label.setText(f"V_min: {v_min}")
        self.h_max_label.setText(f"H_max: {h_max}")
        self.s_max_label.setText(f"S_max: {s_max}")
        self.v_max_label.setText(f"V_max: {v_max}")

        # 定义颜色范围
        lower_color = np.array([h_min, s_min, v_min])
        upper_color = np.array([h_max, s_max, v_max])

        # 读取摄像头图像
        ret, frame = self.cap.read()

        # 将帧转换为HSV颜色空间
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 创建mask图像
        mask = cv2.inRange(hsv_frame, lower_color, upper_color)

        # 将原始图像和mask图像水平拼接在一起
        hconcat_img = cv2.hconcat([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])

        # 将拼接后的图像转换为QImage格式
        qimage = QImage(
            hconcat_img.data,
            hconcat_img.shape[1],
            hconcat_img.shape[0],
            QImage.Format_RGB888,
        ).rgbSwapped()

        # 将QImage显示在标签上
        # self.image_label.setPixmap(QPixmap.fromImage(qimage).scaled((self.width)*2, (self.height)))
        self.mask_label.setPixmap(
            QPixmap.fromImage(qimage).scaled((self.width) * 2, (self.height))
        )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())
