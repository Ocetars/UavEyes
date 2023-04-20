import cv2
import numpy as np

# 创建一个窗口来显示图像和指示
cv2.namedWindow ("Image")

# 打开摄像头
cap = cv2.VideoCapture (0)

# 循环处理每一帧图像
while True:
    # 读取一帧图像
    ret, frame = cap.read ()
    if not ret:
        break
    
    # 转换为灰度图像
    gray = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)
    
    # 二值化处理
    _, thresh = cv2.threshold (gray, 100, 255, cv2.THRESH_BINARY)
    
    # 寻找呼啦圈轮廓
    contours, _ = cv2.findContours (thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 如果找到了轮廓
    if contours:
        # 取最大的轮廓
        cnt = max (contours, key=cv2.contourArea)
        
        # 拟合一个椭圆
        ellipse = cv2.fitEllipse (cnt)
        
        # 在原始图像上绘制椭圆
        cv2.ellipse (frame, ellipse, (0, 255, 0), 2)
        
        # 获取椭圆的参数
        center = ellipse [0] # 中心坐标
        axes = ellipse [1] # 主轴和次轴长度
        angle = ellipse [2] # 旋转角度
        
        # 显示椭圆的参数
        cv2.putText (frame, "Center: {:.0f}, {:.0f}".format (center [0], center [1]), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText (frame, "Axes: {:.0f}, {:.0f}".format (axes [0], axes [1]), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText (frame, "Angle: {:.0f}".format (angle), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # 根据椭圆的参数给出指示
        instruction = ""
        if angle > 5: # 如果旋转角度大于5度，说明需要向左旋转摄像头
            instruction += "Rotate left "
        elif angle < -5: # 如果旋转角度小于-5度，说明需要向右旋转摄像头
            instruction += "Rotate right "
        
        if center [0] < frame.shape [1] / 2 - 20: # 如果中心坐标的x值小于图像宽度的一半减20，说明需要向右移动摄像头
            instruction += "Move right "
        elif center [0] > frame.shape [1] / 2 + 20: # 如果中心坐标的x值大于图像宽度的一半加20，说明需要向左移动摄像头
            instruction += "Move left "
        
        if center [1] < frame.shape [0] / 2 - 20: # 如果中心坐标的y值小于图像高度的一半减20，说明需要向下移动摄像头
            instruction += "Move down "
        elif center [1] > frame.shape [0] / 2 + 20: # 如果中心坐标的y值大于图像高度的一半加20，说明需要向上移动摄像头
            instruction += "Move up "
        
        if instruction == "": # 如果没有任何指示，说明已经达到目标状态
            instruction = "Done!"
        
        # 显示指示
        cv2.putText (frame, instruction, (10, frame.shape [0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
    
    # 显示图像和指示
    cv2.imshow ("Image", frame)
    
    # 按ESC键退出循环
    key = cv2.waitKey (1) & 0xFF
    if key == 27:
        break

# 关闭摄像头和窗口
cap.release ()
cv2.destroyAllWindows ()