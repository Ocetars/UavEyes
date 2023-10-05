import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk

# Load the image using OpenCV
image = cv2.imread("./2023/way.png")
image2 = cv2.imread("./2023/864x720.jpg")
resized_image = cv2.resize(image, (1280, 720))
# Convert the image to RGB format
resized_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
resized_image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)
# Convert the image to PIL format
pil_image = Image.fromarray(resized_image)
pil_image2 = Image.fromarray(resized_image2)
# 自定义开关
external_data = "tab2"


root = tk.Tk()
# Set the window size
root.geometry("1280x720")
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

label1 = tk.Label(tab2,text="无人机位置坐标", font=("Arial", 20), justify="center")
label1.pack(side="top", pady=(60,10), padx=10)
entry1 = tk.Entry(tab2, font=("Arial", 20), justify="center")
entry1.pack(side="top", pady=(10,10), padx=10)

label2 = tk.Label(tab2, text="累计巡逻航程", font=("Arial", 20))
label2.pack(side="top", pady=(60,10), padx=10)
entry2 = tk.Entry(tab2, font=("Arial", 20), justify="center")
entry2.pack(side="top", pady=(10,10), padx=10)

label3 = tk.Label(tab2, text="火源地点位置坐标", font=("Arial", 20),fg="red")
label3.pack(side="top", pady=(60,10), padx=10)
entry3 = tk.Entry(tab2, font=("Arial", 20), justify="center")
entry3.pack(side="top", pady=(10,10), padx=10)

# Insert text into the entry widgets
message1 = "0"
message2 = "0"
message3 = "0"
entry1.insert(0, message1)
entry2.insert(0, message2)
entry3.insert(0, message3)

# Determine which tab to show
if external_data == "tab1":
    notebook.select(tab1)
else:
    notebook.select(tab2)

# root.attributes('-fullscreen', True)
root.mainloop()