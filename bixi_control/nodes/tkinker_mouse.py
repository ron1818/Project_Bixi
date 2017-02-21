import Tkinter as tk
root = tk.Tk()

root.geometry('{}x{}'.format(1300, 700))

linear=0
steer=0
max_linear=50
max_steer=50
previous_x=0
previous_y=0

#Cs style

def mouseMotion(event):
    global previous_y, previous_x
    del_x=0
    del_y=0
    x, y = event.x, event.y
    current_x, current_y= x, y
    del_x+=current_x-previous_x
    del_y+=current_y-previous_y
    previous_x, previous_y= x, y

    print('{}, {}'.format(del_x, del_y))

def leftClick(event):
    print("left, shoot")

def midClick(event):
    print("mid, deactivate")

def rightClick(event):
    print("right, activate")

def keyBoard(event):
    print(event.char)
    global steer, linear

    if event.char=='a':
        #left
        steer-=1

    elif event.char=='w':
        #up
        linear+=1
    elif event.char=='d':
        #right
        steer+=1
    elif event.char=='s':
        #down
        linear-=1

    if abs(linear)>max_linear:
        linear=max_linear*linear/abs(linear)

    if abs(steer)>max_steer:
        steer=max_steer*steer/abs(steer)

    print(linear, steer)

frame=tk.Frame(root, width=1300, height=700)
frame.bind('<Button-1>', leftClick)
frame.bind('<Button-2>', midClick)
frame.bind('<Button-3>', rightClick)
frame.bind('<Motion>', mouseMotion)

frame.pack()
root.bind('<Key>', keyBoard)
decay_rate=0.2


root.mainloop()

while True:
    print(linear, steer)
    if linear!=0:
        linear=linear-linear*decay_rate/abs(linear)
    if steer!=0:
        steer=steer-steer*decay_rate/abs(steer)





