import Tkinter as tk
root = tk.Tk()


abs_x=0
abs_y=0
previous_x=0
previous_y=0

scaler=float(0.2)/14095
#integrate to odom

def motion(event):
    global abs_x, abs_y, previous_y, previous_x, scaler

    x, y = event.x, event.y
    current_x, current_y= x, y
    abs_x+=current_x-previous_x
    abs_y+=current_y-previous_y
    previous_x, previous_y= x, y

    if x>1200 or y>650 or x<100 or y<100:
        previous_x=650
        previous_y=350
    	root.event_generate('<Motion>', warp=True, x=650, y=350)


    print('{}, {}'.format(abs_x*scaler, abs_y*scaler))

root.bind('<Motion>', motion)
root.mainloop()