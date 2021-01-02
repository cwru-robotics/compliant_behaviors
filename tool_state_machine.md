# State machine to do tool stowage:

## Knowns:
* Task frame (when tool is fully stowed)
* Height/pose of upper stop (before engagement of the first lock)



## Commands to send
1. Move down until target distance reached (persists along multiple commands, from some start pose)
2. Rotate to some target orientation (persists between multiple commands from some start pose)
3. Relieve loads (if commands are unable to process)
4. Wiggle If a series of commands sees no progress (after trying relieving and moving both fail)

## States:
1. Move down from above the stowage bay (to the upper stop)
    * Exit conditions
        1. Z axis of the tool is within some error of the initial z axis OR stowage bay z axis (task frame), parallel to the task (potential) vector comparison, assuming we have a priori knowledge of the storage bay
        2. Tool is within some error in the x,y, and z position to the tool (cylinder shaped accepted area), the upper stop section of the tool stowage bay [insert photo of tool sitting on top here]
        3. Wrench limit breached along main axis for force, other axes are minimal
    * Actions
        - Move down until 1 and 2 exit conditions are met (Command 1)
        - If movement is sent 2-3 times and no significant progress in movement dir then try
            - Wiggle if no wrench breach or no progress after relieving loads and sending move commands (command 4)
            - Relieve Loads if wrench breached (command 3) tentative and up for interpretation and changes
        - If 1 and 2 are met, relieve loads (command 3) until all 3 are met
        - If all 3 are met, transition to the next state
2. Rotate the locks open
    * Exit Conditions:
        1. Wrench Breach
        2. Orientation Target
    * Actions:
        - Rotate along z axis
        - Relieve Loads (all loads but torque in z)
        - Stop when conditions met, then relieve loads
3. Move down to bottom of tool stowage
    * Exit conditions
        1. Z axis of the tool is within some error of the initial z axis OR stowage bay z axis (task frame), parallel to the task (potential) vector comparison, assuming we have a priori knowledge of the storage bay
        2. Tool is within some error in the x,y, and z position to the tool (cylinder shaped accepted area), the bottom section of the tool stowage bay [insert photo of tool sitting on top here]
        3. Wrench limit breached along main axis for force, other axes are minimal (Optional, we want to reduce wrench/loads on axes other than force in z, expect to have torque in z)
    * Actions
        - Move down until 1 and 2 exit conditions are met (Command 1)
        - If movement is sent 2-3 times and no significant progress in movement dir then try
            - Wiggle if no wrench breach or no progress after relieving loads and sending move commands (command 4)
            - Relieve Loads if wrench breached (command 3) tentative and up for interpretation and changes
        - If 1 and 2 are met, relieve loads (command 3) until all 3 are met
        - If all 3 are met, transition to the next state
4. Rotate into the locked position
    * Exit Conditions:
        1. Wrench Breach
        2. Orientation Target, position target (tool frame almost aligned with task frame)
    * Actions:
        - Rotate along z axis
        - Relieve Loads (all loads but torque in z, and force in z)
        - Stop when conditions met, then relieve loads


## Commands machine to do tool retrieval:
1. Move up until target distance reached (persists along multiple commands, from some start pose)
2. Rotate to some target orientation (persists between multiple commands from some start pose)
3. Relieve loads (if commands are unable to process)
4. Wiggle If a series of commands sees no progress (after trying relieving and moving both fail)

## States:
1. Rotate to the unlocked position
    * Exit Conditions:
        1. Wrench Breach
        2. Orientation Target
    * Actions:
        - Rotate along z axis
        - Relieve Loads (all loads but torque in z)
        - Stop when conditions met, then relieve loads (except torque in z)
2. Move up
    * Exit Conditions
        - Some delta above the task frame (clear of the stowage bay)
        - Minimal to 0 loads (free space, but also now holding tool)
    * Actions
        - Move up if not within position target
        - Relieve Loads if stuck or wrench limit crossed
        - Wiggle if no translation or rotation progress
        - If orientation has rotated back towards lock position, revert back to state 1