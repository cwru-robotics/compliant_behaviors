#!/usr/bin/python
# -*- coding: utf-8 -*-
import Tkinter as tk
import ttk

# import roslaunch

import os
import time

# Length of the lag between sending and the machine receiving emssage
delay_len = 0

# List of options for preset values, to be associated with related values in each function
options = ['Peg', 'Bottle_Cap', 'Cutting', 'Tool', 'Task', 'Stowage']
selected_option = 4

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

        self.master.title('CartP2P GUI')
        self.tab_control = ttk.Notebook(self.master)
        self.motion_control = ttk.Frame(self.tab_control)
        self.wiggle_control = ttk.Frame(self.tab_control)
        self.rwe_control = ttk.Frame(self.tab_control)
        self.joint_control = ttk.Frame(self.tab_control)

        self.tab_control.add(self.motion_control, text='Motion Control')
        self.tab_control.add(self.wiggle_control, text='Dither Control')
        self.tab_control.add(self.rwe_control, text='RWE Control')
        self.tab_control.add(self.joint_control, text='Joint Control')

        # Pack and display the tabs

        self.tab_control.pack(expand=1, fill='both')

        # Font defined to be larger
        # myFont = tk.font.Font(size=20, weight='bold')

        # Config for the button layout

        self.quit = tk.Button(self, text='QUIT', fg='red',
                              command=self.master.destroy,
                              font='Courier 20 bold')

        # self.quit['font'] = myFont

        self.quit.grid(row=0, column=0, sticky=tk.W)

        self.entry = tk.Entry(self, font='Courier 20 bold')
        self.entry.grid(row=0, column=1, sticky=tk.NSEW)

        self.clear_text = tk.Button(self, text='Clear Text', fg='blue',
                                    command=self.clear_text,
                                    font='Courier 20 bold')
        self.clear_text.grid(row=0, column=2, sticky=tk.E)

        #Parameter Dropdown Menu

        # This variable stores the value of the set of parameters needed, will send it to the ptfl
        self.parameter_set = tk.StringVar(self)
        self.parameter_set.set(options[selected_option]) # Change the number in the brackets for which one 

        # Comented out is if I want to link a function to the update of the menu
        self.drop = tk.OptionMenu(self, self.parameter_set, *options) #, font='Courier 20 bold') # command=self.updated_menu, 
        self.drop.grid(row=0,column=4,sticky=tk.W+tk.E)
        self.drop.config(font=('courier',(20)))

        self.param_label = \
            tk.Label(self,
                     text=' Preset Parameters:',
                     font='Courier 20 bold')
        self.param_label.grid(row=0, column=3,
                sticky=tk.NSEW, pady=5, padx=2)


        #! Force Moment Accomodation (Present in both motion and orientation control tabs)
        # MOTION CONTROL FMA
        self.restore_wrench_eq_label = tk.Label(self.rwe_control,
                text='Limp Mode:',
                font='Courier 20 bold')
        self.restore_wrench_eq_label.grid(row=5, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        self.restore_wrench_eq = tk.Button(self.rwe_control,
                text='text in', fg='blue',
                command=self.restore_wrench_eq, font='Courier 20 bold')
        self.restore_wrench_eq.grid(row=5, column=2, sticky=tk.NSEW,
                                   pady=5, padx=2)

        # Preset Value 1

        self.restore_wrench_eq_preset_1 = tk.Button(self.rwe_control,
                text='10 sec', fg='blue',
                command=self.restore_wrench_eq_preset_1,
                font='Courier 20 bold')
        self.restore_wrench_eq_preset_1.grid(row=5, column=4,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset Value 2

        self.restore_wrench_eq_preset_2 = tk.Button(self.rwe_control,
                text='5 sec', fg='blue',
                command=self.restore_wrench_eq_preset_2,
                font='Courier 20 bold')
        self.restore_wrench_eq_preset_2.grid(row=5, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        #! Cart P2P 

        # Label and Submission Button
        self.cartp2p_label = tk.Label(self.motion_control,
                                               text='Cart P2PTWL:',
                                               font='Courier 20 bold')
        self.cartp2p_label.grid(row=2, column=2,
                                         sticky=tk.NSEW, pady=5, padx=2)

        self.cartp2p = tk.Button(self.motion_control,
                                          text='Send cmd', fg='blue',
                                 command=self.cartp2p_fnc, font='Courier 20 bold')
        self.cartp2p.grid(row=2, column=3, sticky=tk.NSEW,
                                   pady=5, padx=2)

        # Label for translation and rotation units
        self.trans_label = tk.Label(self.motion_control,
                                               text='Trans in m',
                                               fg='green',
                                               font='Courier 20 bold')
        self.trans_label.grid(row=5, column=0,
                                         sticky=tk.NSEW, pady=5, padx=2)

        self.rot_label = tk.Label(self.motion_control,
                                               text='Rot in rads',
                                               fg='green',
                                               font='Courier 20 bold')
        self.rot_label.grid(row=5, column=5,
                                         sticky=tk.NSEW, pady=5, padx=2)

        # Labeled inputs for cartp2p
        self.cartp2p_label = tk.Label(self.motion_control,
                                               text='Trans x',
                                               font='Courier 20 bold')
        self.cartp2p_label.grid(row=3, column=0,
                                         sticky=tk.NSEW, pady=10, padx=2)

        self.cartp2p_label = tk.Label(self.motion_control,
                                               text='Trans y',
                                               font='Courier 20 bold')
        self.cartp2p_label.grid(row=3, column=1,
                                         sticky=tk.NSEW, pady=10, padx=2)
                                        
        self.cartp2p_label = tk.Label(self.motion_control,
                                               text='Trans z',
                                               font='Courier 20 bold')
        self.cartp2p_label.grid(row=3, column=2,
                                         sticky=tk.NSEW, pady=10, padx=2)
        self.cartp2p_label = tk.Label(self.motion_control,
                                               text='Rot x',
                                               font='Courier 20 bold')
        self.cartp2p_label.grid(row=3, column=3,
                                         sticky=tk.NSEW, pady=10, padx=20)

        self.cartp2p_label = tk.Label(self.motion_control,
                                               text='Rot y',
                                               font='Courier 20 bold')
        self.cartp2p_label.grid(row=3, column=4,
                                         sticky=tk.NSEW, pady=10, padx=20)
                                        
        self.cartp2p_label = tk.Label(self.motion_control,
                                               text='Rot z',
                                               font='Courier 20 bold')
        self.cartp2p_label.grid(row=3, column=5,
                                         sticky=tk.NSEW, pady=10, padx=20)

        # Fields for the related entries
        self.cart_trans_x = tk.Entry(self.motion_control, width=12, font='Courier 20 bold')
        self.cart_trans_x.grid(row=4, column=0, sticky=tk.NSEW, pady=5, padx=2)

        self.cart_trans_y = tk.Entry(self.motion_control, width=12, font='Courier 20 bold')
        self.cart_trans_y.grid(row=4, column=1, sticky=tk.NSEW, pady=5, padx=2)

        self.cart_trans_z = tk.Entry(self.motion_control, width=12, font='Courier 20 bold')
        self.cart_trans_z.grid(row=4, column=2, sticky=tk.NSEW, pady=5, padx=2)

        self.cart_rot_x = tk.Entry(self.motion_control, width=12, font='Courier 20 bold')
        self.cart_rot_x.grid(row=4, column=3, sticky=tk.NSEW, pady=5, padx=2)

        self.cart_rot_y = tk.Entry(self.motion_control, width=12, font='Courier 20 bold')
        self.cart_rot_y.grid(row=4, column=4, sticky=tk.NSEW, pady=5, padx=2)

        self.cart_rot_z = tk.Entry(self.motion_control, width=12, font='Courier 20 bold')
        self.cart_rot_z.grid(row=4, column=5, sticky=tk.NSEW, pady=5, padx=2)


        self.cart_bumpless = ttk.Checkbutton(self.motion_control, text='Bumpless')
        self.cart_bumpless.grid(row=2, column=0, sticky=tk.NSEW, pady=5, padx=2)

        #! Joint Control UI
        
        # Cart P2P Label and Submission Button
        self.joint_label = tk.Label(self.joint_control,
                                               text='Joint Cmdr:',
                                               font='Courier 20 bold')
        self.joint_label.grid(row=2, column=2,
                                         sticky=tk.NSEW, pady=5, padx=2)

        self.joint_cmd = tk.Button(self.joint_control,
                                          text='Send cmd', fg='blue',
                                 command=self.joint_cmd_fnc, font='Courier 20 bold')
        self.joint_cmd.grid(row=2, column=3, sticky=tk.NSEW,
                                   pady=5, padx=2)

        # Preset buttons for joint poses
        self.joint_cmd_pre_home = tk.Button(self.joint_control,
                                          text='Home Pose', fg='blue',
                                 command=self.simple_joint_commander_pre_home_pose, font='Courier 20 bold')
        self.joint_cmd_pre_home.grid(row=5, column=1, sticky=tk.NSEW,
                                   pady=5, padx=2)

        self.joint_cmd_pull_back = tk.Button(self.joint_control,
                                          text='Pull-back Pose', fg='blue',
                                 command=self.simple_joint_commander_pre_pull_back_pose, font='Courier 20 bold')
        self.joint_cmd_pull_back.grid(row=5, column=2, sticky=tk.NSEW,
                                   pady=5, padx=2)

        self.joint_cmd_rotate = tk.Button(self.joint_control,
                                          text='Rotate Pose', fg='blue',
                                 command=self.simple_joint_commander_pre_rotate_pose, font='Courier 20 bold')
        self.joint_cmd_rotate.grid(row=5, column=3, sticky=tk.NSEW,
                                   pady=5, padx=2)

        self.joint_cmd_qd = tk.Button(self.joint_control,
                                          text='QD Pose', fg='blue',
                                 command=self.simple_joint_commander_pre_qd_pose, font='Courier 20 bold')
        self.joint_cmd_qd.grid(row=5, column=4, sticky=tk.NSEW,
                                   pady=5, padx=2)

        self.joint_cmd_transition = tk.Button(self.joint_control,
                                          text='Transition', fg='blue',
                                 command=self.simple_joint_commander_pre_transition, font='Courier 20 bold')
        self.joint_cmd_transition.grid(row=5, column=0, sticky=tk.NSEW,
                                   pady=5, padx=2)

        self.joint_cmd_zero = tk.Button(self.joint_control,
                                          text='Zero', fg='blue',
                                 command=self.simple_joint_commander_pre_zero_pose, font='Courier 20 bold')
        self.joint_cmd_zero.grid(row=5, column=5, sticky=tk.NSEW,
                                   pady=5, padx=2)

        # Label for translation and rotation units
        self.trans_label = tk.Label(self.joint_control,
                                               text='Angs in degs',
                                               fg='green',
                                               font='Courier 20 bold')
        self.trans_label.grid(row=2, column=0,
                                         sticky=tk.NSEW, pady=5, padx=2)
        # Labeled inputs for Joint Command
        self.joint_label = tk.Label(self.joint_control,
                                               text='Joint 1',
                                               font='Courier 20 bold')
        self.joint_label.grid(row=3, column=0,
                                         sticky=tk.NSEW, pady=10, padx=2)

        self.joint_label = tk.Label(self.joint_control,
                                               text='Joint 2',
                                               font='Courier 20 bold')
        self.joint_label.grid(row=3, column=1,
                                         sticky=tk.NSEW, pady=10, padx=2)
                                        
        self.joint_label = tk.Label(self.joint_control,
                                               text='Joint 3',
                                               font='Courier 20 bold')
        self.joint_label.grid(row=3, column=2,
                                         sticky=tk.NSEW, pady=10, padx=2)
        self.joint_label = tk.Label(self.joint_control,
                                               text='Joint 4',
                                               font='Courier 20 bold')
        self.joint_label.grid(row=3, column=3,
                                         sticky=tk.NSEW, pady=10, padx=20)

        self.joint_label = tk.Label(self.joint_control,
                                               text='Joint 5',
                                               font='Courier 20 bold')
        self.joint_label.grid(row=3, column=4,
                                         sticky=tk.NSEW, pady=10, padx=20)
                                        
        self.joint_label = tk.Label(self.joint_control,
                                               text='Joint 6',
                                               font='Courier 20 bold')
        self.joint_label.grid(row=3, column=5,
                                         sticky=tk.NSEW, pady=10, padx=20)

        # Fields for the related entries
        self.joint_1 = tk.Entry(self.joint_control, width=12, font='Courier 20 bold')
        self.joint_1.grid(row=4, column=0, sticky=tk.NSEW, pady=5, padx=2)

        self.joint_2 = tk.Entry(self.joint_control, width=12, font='Courier 20 bold')
        self.joint_2.grid(row=4, column=1, sticky=tk.NSEW, pady=5, padx=2)

        self.joint_3 = tk.Entry(self.joint_control, width=12, font='Courier 20 bold')
        self.joint_3.grid(row=4, column=2, sticky=tk.NSEW, pady=5, padx=2)

        self.joint_4 = tk.Entry(self.joint_control, width=12, font='Courier 20 bold')
        self.joint_4.grid(row=4, column=3, sticky=tk.NSEW, pady=5, padx=2)

        self.joint_5 = tk.Entry(self.joint_control, width=12, font='Courier 20 bold')
        self.joint_5.grid(row=4, column=4, sticky=tk.NSEW, pady=5, padx=2)

        self.joint_6 = tk.Entry(self.joint_control, width=12, font='Courier 20 bold')
        self.joint_6.grid(row=4, column=5, sticky=tk.NSEW, pady=5, padx=2)


        #! Torsional Wiggle Pull

        self.torsional_wiggle_pull_label = \
            tk.Label(self.wiggle_control, text='Torsional Wiggle Pull:'
                     , font='Courier 20 bold')
        self.torsional_wiggle_pull_label.grid(row=0, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.torsional_wiggle_pull = tk.Button(self.wiggle_control,
                text='text in', fg='blue',
                command=self.torsional_wiggle_pull,
                font='Courier 20 bold')
        self.torsional_wiggle_pull.grid(row=0, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1

        self.torsional_wiggle_pull_pre_1 = \
            tk.Button(self.wiggle_control, text='3 sec', fg='blue',
                      command=self.torsional_wiggle_pull_pre_1,
                      font='Courier 20 bold')
        self.torsional_wiggle_pull_pre_1.grid(row=0, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.torsional_wiggle_pull_pre_2 = \
            tk.Button(self.wiggle_control, text='5 sec', fg='blue',
                      command=self.torsional_wiggle_pull_pre_2,
                      font='Courier 20 bold')
        self.torsional_wiggle_pull_pre_2.grid(row=0, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        #! Torsional Wiggle Push

        self.torsional_wiggle_push_label = \
            tk.Label(self.wiggle_control, text='Torsional Wiggle Push:'
                     , font='Courier 20 bold')
        self.torsional_wiggle_push_label.grid(row=1, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.torsional_wiggle_push = tk.Button(self.wiggle_control,
                text='text in', fg='blue',
                command=self.torsional_wiggle_push,
                font='Courier 20 bold')
        self.torsional_wiggle_push.grid(row=1, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1

        self.torsional_wiggle_push_pre_1 = \
            tk.Button(self.wiggle_control, text='3 sec', fg='blue',
                      command=self.torsional_wiggle_push_pre_1,
                      font='Courier 20 bold')
        self.torsional_wiggle_push_pre_1.grid(row=1, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.torsional_wiggle_push_pre_2 = \
            tk.Button(self.wiggle_control, text='5 sec', fg='blue',
                      command=self.torsional_wiggle_push_pre_2,
                      font='Courier 20 bold')
        self.torsional_wiggle_push_pre_2.grid(row=1, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        #! Translational Wiggle Pull

        self.translational_wiggle_pull_label = \
            tk.Label(self.wiggle_control,
                     text='Translational Wiggle Pull:',
                     font='Courier 20 bold')
        self.translational_wiggle_pull_label.grid(row=2, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.translational_wiggle_pull = tk.Button(self.wiggle_control,
                text='text in', fg='blue',
                command=self.translational_wiggle_pull,
                font='Courier 20 bold')
        self.translational_wiggle_pull.grid(row=2, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1

        self.translational_wiggle_pull_pre_1 = \
            tk.Button(self.wiggle_control, text='3 sec', fg='blue',
                      command=self.translational_wiggle_pull_pre_1,
                      font='Courier 20 bold')
        self.translational_wiggle_pull_pre_1.grid(row=2, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.translational_wiggle_pull_pre_2 = \
            tk.Button(self.wiggle_control, text='5 sec', fg='blue',
                      command=self.translational_wiggle_pull_pre_2,
                      font='Courier 20 bold')
        self.translational_wiggle_pull_pre_2.grid(row=2, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        #! Translational Wiggle Push

        self.translational_wiggle_push_label = \
            tk.Label(self.wiggle_control,
                     text='Translational Wiggle Push:',
                     font='Courier 20 bold')
        self.translational_wiggle_push_label.grid(row=3, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.translational_wiggle_push = tk.Button(self.wiggle_control,
                text='text in', fg='blue',
                command=self.translational_wiggle_push,
                font='Courier 20 bold')
        self.translational_wiggle_push.grid(row=3, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1

        self.translational_wiggle_push_pre_1 = \
            tk.Button(self.wiggle_control, text='3 sec', fg='blue',
                      command=self.translational_wiggle_push_pre_1,
                      font='Courier 20 bold')
        self.translational_wiggle_push_pre_1.grid(row=3, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.translational_wiggle_push_pre_2 = \
            tk.Button(self.wiggle_control, text='5 sec', fg='blue',
                      command=self.translational_wiggle_push_pre_2,
                      font='Courier 20 bold')
        self.translational_wiggle_push_pre_2.grid(row=3, column=3,
                sticky=tk.NSEW, pady=5, padx=2)


        # Show all buttons

        self.pack()

    # def say_hi(self):
    #     print 'hi there, everyone!'

    def clear_text(self):

        self.cart_trans_x.delete(0, 'end')
        self.cart_trans_y.delete(0, 'end')
        self.cart_trans_z.delete(0, 'end')
        self.cart_rot_x.delete(0, 'end')
        self.cart_rot_y.delete(0, 'end')
        self.cart_rot_z.delete(0, 'end')
        self.joint_1.delete(0, 'end')
        self.joint_2.delete(0, 'end')
        self.joint_3.delete(0, 'end')
        self.joint_4.delete(0, 'end')
        self.joint_5.delete(0, 'end')
        self.joint_6.delete(0, 'end')
        self.entry.delete(0, 'end')


    #! Change which file is called here, and add in params for which axes to preserve wrench
    def restore_wrench_eq(self):

        # print("Here, i run a command using os, skill 2")

        command = \
            'rosrun behavior_algorithms force_moment_accommodation_interaction_port'
        run_time = self.parse_entry()
        if run_time > 0:
            command = command + ' _run_time:=%f' % run_time

        print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def restore_wrench_eq_preset_1(self):

        # print("Here, i run a command using os, skill 2")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms force_moment_accommodation_interaction_port _run_time:=10'
                  )  
        time.sleep(delay_len)

    def restore_wrench_eq_preset_2(self):

        # print("Here, i run a command using os, skill 1")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms force_moment_accommodation_interaction_port _run_time:=5'
                  )  
        time.sleep(delay_len)

    
    # New example 
    # rosrun behavior_algorithms orientation_targeting_effort_limiting_y _target_orientation:=0.25

    def cartp2p_fnc(self):

        # print("Here, i run a command using os, skill 2")

        # Fix function call, and accept the inputs from the correct fields
        
        # Parse each input, if it is a 
        # x = y = z = rot_x = rot_y = rot_z = 0
        try:
            x = float(self.cart_trans_x.get())
        except:
            x = 0

        try:
            y = float(self.cart_trans_y.get())
        except:
            y = 0

        try:
            z = float(self.cart_trans_z.get())
        except:
            z = 0

        try:
            rot_x = float(self.cart_rot_x.get())
        except:
            rot_x = 0

        try:
            rot_y = float(self.cart_rot_y.get())
        except:
            rot_y = 0

        try:
            rot_z = float(self.cart_rot_z.get())
        except:
            rot_z = 0

        print(x,y,z,rot_x,rot_y,rot_z)
        bump = False
        if self.cart_bumpless.instate(['selected']):
            bump = 1
        else:
            bump = 0
        command = \
            'rosrun behavior_algorithms cartp2ptwl _trans_x:={0} _trans_y:={1} _trans_z:={2} _rot_x:={3} _rot_y:={4} _rot_z:={5} _param_set:={6} _bumpless:={7}'.format(x,y,z,rot_x,rot_y,rot_z,self.parameter_set.get(),bump)
        

        print(command)

        time.sleep(delay_len)
        os.system(command)
        time.sleep(delay_len)
        
    
    def joint_cmd_fnc(self):

        # print("Here, i run a command using os, skill 2")

        # Fix function call, and accept the inputs from the correct fields
        
        # Parse each input, if it is a 
        # x = y = z = rot_x = rot_y = rot_z = 0
        try:
            joint_1 = float(self.joint_1.get())
        except:
            joint_1 = 0

        try:
            joint_2 = float(self.joint_2.get())
        except:
            joint_2 = -48

        try:
            joint_3 = float(self.joint_3.get())
        except:
            joint_3 = 16

        try:
            joint_4 = float(self.joint_4.get())
        except:
            joint_4 = 0

        try:
            joint_5 = float(self.joint_5.get())
        except:
            joint_5 = 30

        try:
            joint_6 = float(self.joint_6.get())
        except:
            joint_6 = 0

        print(joint_1,joint_2,joint_3,joint_4,joint_5,joint_6)
        
        command = \
            'rosrun irb120_accomodation_control simple_joint_commander _joint_1:={0} _joint_2:={1} _joint_3:={2} _joint_4:={3} _joint_5:={4} _joint_6:={5}'.format(joint_1,joint_2,joint_3,joint_4,joint_5,joint_6)
        

        print(command)

        time.sleep(delay_len)
        os.system(command)
        time.sleep(delay_len)
        
    # Preset to joint command to the home pose above tool stowage
    def simple_joint_commander_pre_home_pose(self):
        time.sleep(delay_len)
        os.system('rosrun irb120_accomodation_control simple_joint_commander _joint_1:=0 _joint_2:=-48 _joint_3:=16 _joint_4:=0 _joint_5:=30 _joint_6:=0')
        time.sleep(delay_len)

    # Preset to joint command to the pull-back position (above the tool stowage)
    def simple_joint_commander_pre_pull_back_pose(self):
        time.sleep(delay_len)
        os.system('rosrun irb120_accomodation_control simple_joint_commander _joint_1:=0 _joint_2:=-57 _joint_3:=25 _joint_4:=0 _joint_5:=30 _joint_6:=0')
        time.sleep(delay_len)

    # Preset to joint command to the rotate-pose
    def simple_joint_commander_pre_rotate_pose(self):
        time.sleep(delay_len)
        os.system('rosrun irb120_accomodation_control simple_joint_commander _joint_1:=0 _joint_2:=-57 _joint_3:=25 _joint_4:=0 _joint_5:=-60 _joint_6:=0')
        time.sleep(delay_len)

    # Preset to joint command to the QD position
    def simple_joint_commander_pre_qd_pose(self):
        time.sleep(delay_len)
        os.system('rosrun irb120_accomodation_control simple_joint_commander _joint_1:=0 _joint_2:=-37 _joint_3:=5 _joint_4:=0 _joint_5:=-60 _joint_6:=0')
        time.sleep(delay_len)

    # Preset to joint command to the zero (calibrated) position
    def simple_joint_commander_pre_zero_pose(self):
        time.sleep(delay_len)
        os.system('rosrun irb120_accomodation_control simple_joint_commander _joint_1:=0 _joint_2:=0 _joint_3:=0 _joint_4:=0 _joint_5:=0 _joint_6:=0')
        time.sleep(delay_len)
        
    # Preset to joint commands to run in a set order
    def simple_joint_commander_pre_transition(self):
        time.sleep(delay_len)

        # Have no/low delays between programs whenc ommanding many at once
        # temp_delay = delay_len
        # delay_len = 0.2
        
        # 1 First go to the home pose
        self.simple_joint_commander_pre_home_pose()
        # 2 Then command the pull back pose
        self.simple_joint_commander_pre_pull_back_pose()
        # 3 Then command the rotate pose
        self.simple_joint_commander_pre_rotate_pose()
        # 4 Then approach the QD pose
        self.simple_joint_commander_pre_qd_pose()
        
        time.sleep(delay_len)   

    def torsional_wiggle_pull(self):

        # print("Here, i run a command using os, skill 2")

        command = 'rosrun behavior_algorithms torsional_wiggle_pull'
        run_time = self.parse_entry()
        if run_time > 0:
            command = command + ' _wiggle_time:=%f' % run_time

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def torsional_wiggle_pull_pre_1(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms torsional_wiggle_pull _wiggle_time:=3'
                  )  
        time.sleep(delay_len)

    def torsional_wiggle_pull_pre_2(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms torsional_wiggle_pull _wiggle_time:=5'
                  )  
        time.sleep(delay_len)

    # rosrun behavior_algorithms torsional_wiggle_push _wiggle_time:=6

    def torsional_wiggle_push(self):

        # print("Here, i run a command using os, skill 2")

        command = 'rosrun behavior_algorithms torsional_wiggle_push'
        run_time = self.parse_entry()
        if run_time > 0:
            command = command + ' _wiggle_time:=%f' % run_time

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def torsional_wiggle_push_pre_1(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms torsional_wiggle_push _wiggle_time:=3'
                  )  
        time.sleep(delay_len)

    def torsional_wiggle_push_pre_2(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms torsional_wiggle_push _wiggle_time:=5'
                  )  
        time.sleep(delay_len)

    # rosrun behavior_algorithms translational_wiggle_pull _wiggle_time:=6

    def translational_wiggle_pull(self):

        # print("Here, i run a command using os, skill 2")

        command = 'rosrun behavior_algorithms translational_wiggle_pull'
        run_time = self.parse_entry()
        if run_time > 0:
            command = command + ' _wiggle_time:=%f' % run_time

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def translational_wiggle_pull_pre_1(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms translational_wiggle_pull _wiggle_time:=3'
                  )  
        time.sleep(delay_len)

    def translational_wiggle_pull_pre_2(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms translational_wiggle_pull _wiggle_time:=5'
                  )  
        time.sleep(delay_len)

    # rosrun behavior_algorithms translational_wiggle_push _wiggle_time:=6

    def translational_wiggle_push(self):

        # print("Here, i run a command using os, skill 2")

        command = 'rosrun behavior_algorithms translational_wiggle_push'
        run_time = self.parse_entry()
        if run_time > 0:
            command = command + ' _wiggle_time:=%f' % run_time

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def translational_wiggle_push_pre_1(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms translational_wiggle_push _wiggle_time:=3'
                  )  
        time.sleep(delay_len)

    def translational_wiggle_push_pre_2(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms translational_wiggle_push _wiggle_time:=5'
                  )  
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

    # os.system("rosrun robotiq_ft_sensor rq_sensor")
    # os.system("roslaunch force_reflector cybernet_testing_setup.launch ")

    app.mainloop()


if __name__ == '__main__':
    main()


            
