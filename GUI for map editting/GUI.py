import tkinter as tk
from tkinter import constants  # Add this import
from PIL import Image, ImageTk, ImageDraw
import os

#Create new window
root = tk.Tk()
root.title("Virtual Wall Creator")

#Create canvas for picture
canvas = tk.Canvas(root, width=1600, height=1000)
canvas.pack()

#Open the image
image = Image.open("/home/phuoc/bot_ws/src/my_bot/maps/map.pgm")
image.save("/home/phuoc/bot_ws/src/my_bot/maps/backup_map.pgm")
image = image.resize((image.width * 5, image.height * 5))
photo = ImageTk.PhotoImage(image)
canvas.create_image(0,0,anchor="nw", image=photo)

#Open BK logo
bk_logo = Image.open("/home/phuoc/bot_ws/src/my_bot/GUI for map editting/bk_logo.png")
bk_logo = bk_logo.resize((bk_logo.width // 2, bk_logo.height // 2))
bk_logo_photo = ImageTk.PhotoImage(bk_logo)
canvas.create_image(1600-30,1000,anchor="se", image=bk_logo_photo)

#Open BKIC logo
bkic_logo = Image.open("/home/phuoc/bot_ws/src/my_bot/GUI for map editting/bkic_logo.png")
bkic_logo = bkic_logo.resize((bkic_logo.width // 3, bkic_logo.height // 3))
bkic_logo_photo = ImageTk.PhotoImage(bkic_logo)
canvas.create_image(1600-bk_logo.width-30,990,anchor="se", image=bkic_logo_photo)

#Draw line using 2 points
start_point = None
end_point = None
line_history=[]
def handle_click(event):
    global start_point, end_point
    if not start_point:
        start_point = (event.x, event.y)
    elif not end_point:
        end_point = (event.x, event.y)
        draw_line()
        reset_points()
def draw_line():
    if start_point and end_point:
        draw = ImageDraw.Draw(image)
        draw.line([start_point, end_point], fill="black", width=5)  # Adjust width as needed
        update_image()
        line_history.append((start_point, end_point))  # Add line to history for undo
def reset_points():
    global start_point, end_point
    start_point = None
    end_point = None
def update_image():
    global photo
    photo = ImageTk.PhotoImage(image)
    canvas.create_image(0, 0,anchor="nw", image=photo)

#Define buttons' commands
def open_original_image():
    global image, photo
    image = Image.open("/home/phuoc/bot_ws/src/my_bot/maps/backup_map.pgm")
    image = image.resize((image.width * 5, image.height * 5))
    photo = ImageTk.PhotoImage(image)
    canvas.create_image(0, 0, anchor="nw", image=photo)
    line_history.clear()  # Clear line history for the new image
def undo_line():
    if line_history:
        last_line = line_history.pop()  # Remove last line from history
        draw = ImageDraw.Draw(image)
        # Draw a white line on top to "erase" the last line
        draw.line(last_line, fill="white", width=5)
        update_image()
def save_image():
    global image
    image = image.resize((image.width // 5, image.height // 5))
    image.save("/home/phuoc/bot_ws/src/my_bot/maps/map.pgm")
canvas.bind("<Button-1>", handle_click)

#Save Button
save_button = tk.Button(root, text="Save Image", width=10,height=5,bd='10',background='green',command=save_image)
save_button.place(x=1300, y=0,anchor="n")

#Undo previous line button
undo_button = tk.Button(root, text="Undo Line", width=10,height=5,bd='10',command=undo_line)
undo_button.place(x=1000, y=0,anchor="nw")

#Reload the original image
reload_button = tk.Button(root, text="Reload Image", width=10,height=5,bd='10',background='Yellow',fg='black',command=open_original_image)
reload_button.place(x=1600, y=0,anchor="ne")

#Exit the GUI button
exit_button = tk.Button(root, text='Exit', width=10,height=5, bd='10',background="red",foreground="black", command=root.destroy)
exit_button.place(x=1600, y=120,anchor="ne")

#Run the GUI
root.mainloop()
