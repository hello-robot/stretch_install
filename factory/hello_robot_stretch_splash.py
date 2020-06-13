#!/usr/bin/env python
import Tkinter as tk
root = tk.Tk()
# show no frame
root.overrideredirect(True)
width = root.winfo_screenwidth()
height = root.winfo_screenheight()
root.geometry('%dx%d+%d+%d' % (width, height, 0, 0))

image_file = "/etc/hello-robot/stretch_splash.png"
image = tk.PhotoImage(file=image_file)
canvas = tk.Canvas(root, height=height, width=width, bg='#FDF1F5')
canvas.create_image(width*0.5, height*0.5, image=image)
canvas.pack()

root.after(5000, root.destroy)
root.mainloop()

