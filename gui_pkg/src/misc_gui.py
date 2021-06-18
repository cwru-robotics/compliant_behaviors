#!/usr/bin/python
# -*- coding: utf-8 -*-
import Tkinter as tk
import ttk

import rospy
from std_srvs import *
from irb120_accomodation_control.srv import set_task_frame as TaskFrameSrv
from robotiq_ft_sensor.srv import sensor_accessor as ZeroForcesSrv
from usb_relay_wsn.srv import SetRelay #as GripperSrv
from usb_relay_wsn.srv import SetRelayRequest #as GripperSrvRequest
from usb_relay_wsn.msg import RelayState as RelayStateMsg
from irb120_accomodation_control.srv import freeze_service as FreezeSrv


# import roslaunch

import os
import time
import numpy

# Length of the lag between sending and the machine receiving emssage
delay_len = 0

# List of options for preset values, to be associated with related values in each function
options = ['Peg', 'Bottle_Cap', 'Cutting', 'Tool', 'Task']
selected_option = 3

class Application(tk.Frame):

    def __init__(self, master=None):

        # super().__init__(master)

        tk.Frame.__init__(self, master)
        self.master = master  # might not be needed?

        # self.pack()

        self.create_widgets()

    def create_widgets(self):

        # self.hi_there = tk.Button(self)
        # self.hi_there["text"] = "Hello World\n(click me)"
        # self.hi_there["command"] = self.say_hi
        # self.hi_there.pack(side="top")

        self.master.title('Misc Commands GUI')
        self.tab_control = ttk.Notebook(self.master)
        # self.misc_commands = ttk.Frame(self.tab_control)

        # self.tab_control.add(self.misc_commands, text='Misc. Commands')

        # # Pack and display the tabs

        # self.tab_control.pack(expand=1, fill='both')

        # Font defined to be larger
        # myFont = tk.font.Font(size=20, weight='bold')

        # Config for the button layout

        # self.quit = tk.Button(self, text='QUIT', fg='red',
        #                       command=self.master.destroy,
        #                       font='Courier 20 bold')

        # # self.quit['font'] = myFont

        # self.quit.grid(row=2, column=0, sticky=tk.NS)


        # Button for Zero Forces
        self.zero_forces = tk.Button(self,
                text='Zero all Forces', fg='blue',
                command=self.zero_forces, font='Courier 20 bold')
        self.zero_forces.grid(row=0, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        # Button for Setting Task Frame
        self.set_task_frame = tk.Button(self,
                text='Set Task Frame', fg='blue',
                command=self.set_task_frame, font='Courier 20 bold')
        self.set_task_frame.grid(row=1, column=0,
                sticky=tk.NSEW, pady=5, padx=2)


        # Button for Open Gripper
        self.open_gripper = tk.Button(self,
                text='Engage Gripper', fg='blue',
                command=self.open_gripper, font='Courier 20 bold')
        self.open_gripper.grid(row=0, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Button for Close Gripper
        self.close_gripper = tk.Button(self,
                text='Release Gripper', fg='blue',
                command=self.close_gripper, font='Courier 20 bold')
        self.close_gripper.grid(row=1, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Button for Position Hold
        self.pos_hold = tk.Button(self,
                text='Position Hold', fg='blue',
                command=self.pos_hold, font='Courier 20 bold')
        self.pos_hold.grid(row=2, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Button for Joint Position Hold
        self.j_pos_hold = tk.Button(self,
                text='Joint Pos Hold', fg='blue',
                command=self.joint_pos_hold, font='Courier 20 bold')
        self.j_pos_hold.grid(row=2, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        # Show all buttons

        self.pack()

    # def say_hi(self):
    #     print 'hi there, everyone!'

    def clear_text(self):

        self.entry.delete(0, 'end')

    def zero_forces(self):
        time.sleep(delay_len)
        # os.system('rosservice call /robotiq_ft_sensor_acc "command_id: 8"')

        print(zero_forces_proxy(command_id=8))

        time.sleep(delay_len)

    def set_task_frame(self):
        time.sleep(delay_len)
        # os.system('rosservice call /task_frame_service')
        
        print(task_frame_proxy())
        

        time.sleep(delay_len)

    def open_gripper(self):
        time.sleep(delay_len)
        # os.system("rosservice call /grip '[{id: 1, state: false}, {id: 2, state: true}]'")
        
        print(gripper_proxy(open_grip_req))
    
        time.sleep(0.5)
        time.sleep(delay_len)

    def close_gripper(self):
        time.sleep(delay_len)
        # os.system("rosservice call /grip '[{id: 1, state: true}, {id: 2, state: false}]'")

        print(gripper_proxy(close_grip_req))

        time.sleep(0.5)
        time.sleep(delay_len)
    
    def pos_hold(self):
        time.sleep(delay_len)
        # os.system('rosservice call /freeze_service')

        print(freeze_proxy())

        time.sleep(delay_len)

    def joint_pos_hold(self):
        time.sleep(delay_len)
        os.system('rosservice call /joint_freeze_service')
        time.sleep(delay_len)


    def parse_entry(self):
        # print("output from text box")
        # print(self.entry.get())
        # Clear text in here potentially? kept for now to show what was the last input
        try:
            return float(self.entry.get())
        except:
            return 0


def main():
    root = tk.Tk()
    app = Application(master=root)
    root.attributes('-topmost', True)
    app.mainloop()


if __name__ == '__main__':
    #Establish proxy to zero forces
    rospy.wait_for_service('robotiq_ft_sensor_acc')
    try:
        zero_forces_proxy = rospy.ServiceProxy('robotiq_ft_sensor_acc',ZeroForcesSrv)
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)


    #Establish proxy to set task frame
    rospy.wait_for_service('task_frame_service')
    try:
        task_frame_proxy = rospy.ServiceProxy('task_frame_service',TaskFrameSrv)
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

    #Establish proxy to control the gripper
    rospy.wait_for_service('grip')
    try:
        gripper_proxy = rospy.ServiceProxy('grip',SetRelay)
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

    #Establish request to open the gripper
    open_grip_req = SetRelayRequest()
    open_grip_req.states = [RelayStateMsg(id=1,state=False), RelayStateMsg(id=2,state=True)]

    #Establish request to open the gripper
    close_grip_req = SetRelayRequest()
    close_grip_req.states = [RelayStateMsg(id=1,state=True), RelayStateMsg(id=2,state=False)]

    #Establish proxy to toggle position hold
    rospy.wait_for_service('freeze_service')
    try:
        freeze_proxy = rospy.ServiceProxy('freeze_service',FreezeSrv)
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)



    main()


            
