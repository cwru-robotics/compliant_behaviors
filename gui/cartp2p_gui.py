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

        self.master.title('Skills GUI')
        self.tab_control = ttk.Notebook(self.master)
        self.motion_control = ttk.Frame(self.tab_control)
        self.wiggle_control = ttk.Frame(self.tab_control)
        # self.misc_commands = ttk.Frame(self.tab_control)

        self.tab_control.add(self.motion_control, text='Motion Control')
        self.tab_control.add(self.wiggle_control, text='Dither Control')
        # self.tab_control.add(self.misc_commands, text='Misc. Commands')

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


        # Force Moment Accomodation (Present in both motion and orientation control tabs)
        # MOTION CONTROL FMA
        self.force_moment_acc_label = tk.Label(self.motion_control,
                text='Limp Mode:',
                font='Courier 20 bold')
        self.force_moment_acc_label.grid(row=5, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        self.force_moment_acc = tk.Button(self.motion_control,
                text='text in', fg='blue',
                command=self.force_moment_acc, font='Courier 20 bold')
        self.force_moment_acc.grid(row=5, column=2, sticky=tk.NSEW,
                                   pady=5, padx=2)

        # Preset Value 1

        self.force_moment_acc_preset_1 = tk.Button(self.motion_control,
                text='10 sec', fg='blue',
                command=self.force_moment_acc_preset_1,
                font='Courier 20 bold')
        self.force_moment_acc_preset_1.grid(row=5, column=4,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset Value 2

        self.force_moment_acc_preset_2 = tk.Button(self.motion_control,
                text='5 sec', fg='blue',
                command=self.force_moment_acc_preset_2,
                font='Courier 20 bold')
        self.force_moment_acc_preset_2.grid(row=5, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        # Cart P2P Label and Submission Button
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
        self.trans_label.grid(row=2, column=0,
                                         sticky=tk.NSEW, pady=5, padx=2)

        self.rot_label = tk.Label(self.motion_control,
                                               text='Rot in rads',
                                               fg='green',
                                               font='Courier 20 bold')
        self.rot_label.grid(row=2, column=5,
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

        # Torsional Wiggle Pull

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

        # Torsional Wiggle Push

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

        # Translational Wiggle Pull

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

        # Translational Wiggle Push

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
        self.entry.delete(0, 'end')


    def force_moment_acc(self):

        # print("Here, i run a command using os, skill 2")

        command = \
            'rosrun behavior_algorithms force_moment_accommodation'
        run_time = self.parse_entry()
        if run_time > 0:
            command = command + ' _run_time:=%f' % run_time

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def force_moment_acc_preset_1(self):

        # print("Here, i run a command using os, skill 2")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms force_moment_accommodation _run_time:=10'
                  )  
        time.sleep(delay_len)

    def force_moment_acc_preset_2(self):

        # print("Here, i run a command using os, skill 1")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms force_moment_accommodation _run_time:=5'
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
        
        command = \
            'rosrun behavior_algorithms ptwl_cartp2p _trans_x:={0} _trans_y:={1} _trans_z:={2} _rot_x:={3} _rot_y:={4} _rot_z:={5} _param_set:={6}'.format(x,y,z,rot_x,rot_y,rot_z,self.parameter_set.get())
        

        print(command)

        time.sleep(delay_len)
        os.system(command)
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

    def zero_forces(self):
        time.sleep(delay_len)
        os.system('rosservice call /robotiq_ft_sensor_acc "command_id: 8"')  
        time.sleep(delay_len)

    def set_task_frame(self):
        time.sleep(delay_len)
        os.system('rosservice call /task_frame_service')  
        time.sleep(delay_len)

    def open_gripper(self):
        time.sleep(delay_len)
        os.system("rosservice call /grip '[{id: 1, state: false}, {id: 2, state: true}]'")
        time.sleep(0.5)
        time.sleep(delay_len)

    def close_gripper(self):
        time.sleep(delay_len)
        os.system("rosservice call /grip '[{id: 1, state: true}, {id: 2, state: false}]'")
        time.sleep(0.5)
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


            
