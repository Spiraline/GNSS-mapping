from tkinter import *
import rospy
from geometry_msgs.msg import Pose
from functools import partial

POS_MAX = 20
POS_MIN = -20
ORI_MAX = 3.59
ORI_MIN = -3.59

class tf_publisher:
    def __init__(self):
        self.gui = Tk()
        self.gui.title("GNSS-Localizer TF Publisher")
        self.gui.geometry("700x320+100+100")
        self.gui.resizable(False, False)

        self.text = ["x", "y", "z", "roll", "pitch", "yaw"]
        self.val = [IntVar() for _ in range(6)]

        self.bar = []
        self.label = []
        self.upbutton = []
        self.downbutton = []

        for i in range(6):
            if i < 3:
                self.bar.append(Scale(self.gui, variable=self.val[i],
                    command=partial(self.trackbar, i), orient="horizontal",
                    from_=POS_MIN * 100, to=POS_MAX * 100, length=500, showvalue=False))
            else:
                self.bar.append(Scale(self.gui, variable=self.val[i],
                    command=partial(self.trackbar, i), orient="horizontal",
                    from_=ORI_MIN * 100, to=ORI_MAX * 100, length=500, showvalue=False))
            
            self.bar[-1].place(x=20, y=i*50+10)
            
            self.label.append(Label(self.gui, text= self.text[i] + ": 0.0"))
            self.label[-1].place(x=530, y=i*50+10)

            self.upbutton.append(Button(self.gui, text="▲",
                command=partial(self.up_button, i)))
            self.upbutton[-1].place(x=610, y=i*50)

            self.downbutton.append(Button(self.gui, text="▼",
                command=partial(self.down_button, i)))
            self.downbutton[-1].place(x=650, y=i*50)

        self.tf_pub = rospy.Publisher('gnss_to_localizer', Pose, queue_size=1)

        self.gui.mainloop()

    def trackbar(self, mode, window):
        self.label[mode].config(text=self.text[mode] + ": " + str(self.val[mode].get() / 100))
        self.publish_tf()

    def up_button(self, mode):
        if mode < 3 and self.val[mode].get() >= POS_MAX * 100:
            return
        
        if mode >= 3 and self.val[mode].get() >= ORI_MAX * 100:
            return
        
        self.val[mode].set(self.val[mode].get() + 1)
        self.label[mode].config(text=self.text[mode] + ": " + str(self.val[mode].get() / 100))

        self.publish_tf()

    def down_button(self, mode):
        if mode < 3 and self.val[mode].get() <= POS_MIN * 100:
            return
        
        if mode >= 3 and self.val[mode].get() <= ORI_MIN * 100:
            return
        
        self.val[mode].set(self.val[mode].get() - 1)
        self.label[mode].config(text=self.text[mode] + ": " + str(self.val[mode].get() / 100))

        self.publish_tf()

    def publish_tf(self):
        pose_msg = Pose()
        pose_msg.position.x = self.val[0].get() / 100
        pose_msg.position.y = self.val[1].get() / 100
        pose_msg.position.z = self.val[2].get() / 100
        pose_msg.orientation.x = self.val[3].get() / 100
        pose_msg.orientation.y = self.val[4].get() / 100
        pose_msg.orientation.z = self.val[5].get() / 100

        self.tf_pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('tf_publisher')
    window = tf_publisher()