from Tkinter import *

root = Tk()

def key(event):
    root.event_generate('<Motion>', warp=True, x=50, y=50)

def motion(event):
    print('motion {}, {}'.format(event.x, event.y))

root.bind('<Key>', key)
root.bind('<Motion>', motion)
root.mainloop()