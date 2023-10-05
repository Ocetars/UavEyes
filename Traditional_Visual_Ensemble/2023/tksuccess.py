import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk


def Coordinate2023(xmeter: float, ymeter: float) -> float:
    width, height = 1280, 720
    x0 = xmeter * 10.0 * 18.0 + 63.0
    y0 = ymeter * 10.0 * 18.0 + 63.0
    a = x0
    b = y0
    x1 = -b
    y1 = a
    x = x1
    y = height - y1
    return int(x), int(y)


def orbit2023(img):
    # x, y = receive_ROS()
    x, y = input("请输入坐标点(x,y):").split(",")
    x, y = float(x), float(y)
    # 将坐标点转换为图像坐标系中的坐标
    img_x, img_y = Coordinate2023(x, y)
    # 在图像上绘制坐标点
    imgorbit = cv2.circle(
        img, (img_x, img_y), radius=7, color=(255, 0, 0), thickness=-1
    )
    return imgorbit


def update_image():
    global first
    global image_processd
    # Open the image using OpenCV
    if first:
        image = cv2.imread("./2023/864x720.jpg")
        image_processd = orbit2023(image)
        first = False
    else:
        image_processd = orbit2023(image_processd)
    # Convert the image from OpenCV to PIL format
    image = cv2.cvtColor(image_processd, cv2.COLOR_BGR2RGB)
    image = Image.fromarray(image)
    # Create a Tkinter PhotoImage from the PIL image
    photo = ImageTk.PhotoImage(image)
    # Update the label with the new image
    my_label2.config(image=photo)
    my_label2.image = photo
    # Determine which tab to show
    if external_data == "tab1":
        notebook.select(tab1)
    else:
        notebook.select(tab2)
    # Call this function again in 1000 milliseconds
    root.after(100, update_image)


# For the first tab image
image = cv2.imread("./2023/way.png")
resized_image = cv2.resize(image, (1280, 720))
resized_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
pil_image = Image.fromarray(resized_image)
# For the second tab image
image2 = cv2.imread("./2023/864x720.jpg")
image2_rgb = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)
pil_image2 = Image.fromarray(image2_rgb)
# 自定义开关
external_data = "tab2"
first = True
orbit = False


root = tk.Tk()
# Set the window size
root.geometry("1280x720")
# Create a custom style for the notebook
style = ttk.Style()
style.configure('TNotebook.Tab', padding=(30, 10), font=('Arial', 16))
# Create a notebook with two tabs
notebook = ttk.Notebook(root)
notebook.pack()

# Create the first tab
tab1 = ttk.Frame(notebook)
notebook.add(tab1, text="Tab 1")
# Load the image
Imagekk = ImageTk.PhotoImage(pil_image)
# Create a label with the image in the second tab
my_label = tk.Label(tab1, image=Imagekk)
my_label.pack(side="bottom")
# Create the second tab22222222222222222222222222222222222222222222222222
tab2 = ttk.Frame(notebook)
notebook.add(tab2, text="Tab 2")
# Load the image
image2 = ImageTk.PhotoImage(pil_image2)
my_label2 = tk.Label(tab2, image=image2)
my_label2.pack(side="left")
# Update the label with the new image
my_label2.config(image=image2)
my_label2.image = image2

# Call the update_image function to start the loop
if orbit:
    update_image()

label1 = tk.Label(tab2, text="无人机位置坐标", font=("Arial", 20), justify="center")
label1.pack(side="top", pady=(60, 10), padx=10)
entry1 = tk.Entry(tab2, font=("Arial", 20), justify="center")
entry1.pack(side="top", pady=(10, 10), padx=10)

label2 = tk.Label(tab2, text="累计巡逻航程", font=("Arial", 20))
label2.pack(side="top", pady=(60, 10), padx=10)
entry2 = tk.Entry(tab2, font=("Arial", 20), justify="center")
entry2.pack(side="top", pady=(10, 10), padx=10)

label3 = tk.Label(tab2, text="火源地点位置坐标", font=("Arial", 20), fg="red")
label3.pack(side="top", pady=(60, 10), padx=10)
entry3 = tk.Entry(tab2, font=("Arial", 20), justify="center")
entry3.pack(side="top", pady=(10, 10), padx=10)

# Insert text into the entry widgets
message1 = "0"
message2 = "0"
message3 = "0"
entry1.insert(0, message1)
entry2.insert(0, message2)
entry3.insert(0, message3)

# Create the second tab333333333333333333333333333333333333333333333333333333 
tab3 = ttk.Frame(notebook)
notebook.add(tab3, text="takeoff")
rospy.init_node('my_node', anonymous=True)
# Define the callback functions
def publish_common():
    pub = rospy.Publisher('takeoff_topic', String, queue_size=10)
    pub.publish("common")

def publish_harder():
    pub = rospy.Publisher('takeoff_topic', String, queue_size=10)
    pub.publish("harder")
# Create the two buttons
button1 = ttk.Button(tab3, text="common", command=publish_common,width=20, padding=30)
button2 = ttk.Button(tab3, text="harder", command=publish_harder,width=20, padding=30)

# Add the buttons to the third tab
button1.pack(pady=80)
button2.pack(pady=80)

# root.attributes('-fullscreen', True)
root.mainloop()
