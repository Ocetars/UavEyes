import cv2
from pyzbar import pyzbar

cap = cv2.VideoCapture(2)
# 设置摄像头分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 获取自适配阈值
    binary, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    binary, mat = cv2.threshold(gray, binary, 255, cv2.THRESH_BINARY)
    # pyzbar检测条形码
    barcodes = pyzbar.decode(mat)
    for barcode in barcodes:
        # 提取条形码的边界框坐标
        (x, y, w, h) = barcode.rect
        # 在图像中绘制边界框和条形码数据
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        text = "{} ({})".format(barcodeData, barcodeType)
        cv2.putText(
            frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
        )

    cv2.imshow("Barcode Scanner", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
