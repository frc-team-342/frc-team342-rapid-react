# read me

java\frc\robot

### commands:
the commands folder holds more folders with the actual commands 
needed for the robot to function as needed during competion. 
this includes climbing, driving, shooting, and more. these 
commands call subsystems and each other when needed. 
[more info about command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html)

### subsystems:
the subsystem folder holds the subsystems used within the 
commands in the commands folder. this includes climb, drive, 
intake, and outtake. these subsystems and their methods are 
called outside of the subsystems folder in order to have more 
organized code.



### vision:
the vision folder includes [limeLight](https://docs.limelightvision.io/en/latest/) and [photonVision](https://docs.photonvision.org/en/latest/), which 
the robot uses to target the cargo and the hub.

### constants:
this is where all of the constants are stored, such as the ID 
numbers of different parts of the robot, such as motors.

### main:
this is the place of communication between the compiler and 
the robot. do not modify this code.

### robot:
this commands the very basis of the robot, such as 
initialization and the process of switching from autonomous 
and teleop.

### robotContainer:
this is where the structure of the robot is declared, as well as subsystems being instantiated.