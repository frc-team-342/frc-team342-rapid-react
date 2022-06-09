// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Constants for robot check
    public static final double VOLTAGE_THRESHOLD = 4.5;
    public static final int ROBOT_ANALOG_CHECKER_CHANNEL = 0;

    public final static class DriveConstants {
        // Motor IDs
        public static final int FRONT_LEFT_MOTOR = 1;
        public static final int BACK_LEFT_MOTOR = 2;
        public static final int FRONT_RIGHT_MOTOR = 3;
        public static final int BACK_RIGHT_MOTOR = 4;

        // Speeds for different modes
        public static final double TURBO_SPEED = 1.0;
        public static final double NORMAL_SPEED = 0.8;
        public static final double SLOW_SPEED = 0.4;

        /** Maximum current that each motor can draw without triggering breakers. */
        public static final int CURRENT_LIMIT = 40;

        /** Nominal voltage to be maintained for drive motors. */
        public static final double NOMINAL_VOLTAGE = 12;

        /** Minimum time to reach maximum velocity in seconds when in open-loop control (not PID). */
        public static final double RAMP_RATE = 0.2; 

        /** Wheel diameter in meters. */
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(6 * Math.PI);

        /** Wheel base is the horizontal distance between the center of the back wheel and the center of the front wheel. */
        public static final double WHEEL_BASE = Units.inchesToMeters(20.5);

        /** Track width is the distance between the centerlines of the left and right wheels. */
        public static final double TRACK_WIDTH = Units.inchesToMeters(26.22);

        /** The maximum speed of the robot used during trajectory following. */
        public static final double MAX_SPEED = 1.0;

        /** The maximum acceleration of the robot used during trajectory following. */
        public static final double MAX_ACCELERATION = 1.0;

        /** Maximum angular speed in radians per second squared. */
        public static final double MAX_ROTATION_SPEED = 1.0;

        /** Maximum angular acceleration in radians per second squared. */
        public static final double MAX_ROTATION_ACCELERATION = 1.0;

        /** Distance from the center of the robot to each of the wheels. */
        public static final MecanumDriveKinematics KINEMATICS = new MecanumDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), 
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), 
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), 
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        );
    }

    public static final class VisionConstants {
        
        /** 
         * Height of cam subtracted from height of the hub (converted to meters) 
         * */ 
        public static final double CAM_HEIGHT = Units.inchesToMeters(31.15); 
        
        public static final double TARGET_HEIGHT = Units.inchesToMeters(72.85); 

        public static final double CAM_ANGLE = 45; // Unit is degrees
    }

    public static final class OuttakeConstants {
        public static final int SHOOT_MOTOR_1 = 5;
        public static final int SHOOT_MOTOR_2 = 6;
        public static final int FEEDER_MOTOR = 7;

        /** Reverse speed for cargo removal */
        public static final double REVERSE_SPEED = -0.4;

        public static final double LOAD_SPEED = 0.8;

        /** Counts per revolution for the Falcon encoder. */
        public static final double CPR = 2048;

        /** RPM to shoot for low hub. */
        public static final double LOW_RPM = 2075.0f;

        /** RPM to shoot for upper hub. */
        public static final double HIGH_RPM = 3975.0f; 

        /** RPM tolerance for being considered up to speed. */
        public static final double RPM_ERROR = 45;

        /** P constant for the shooter PID loop. */
        public static final double P = 0.01;

        /** I constant for the shooter PID Loop. */
        public static final double I = 1e-4;

        /** D constant for the shooter PID loop. */
        public static final double D = 0.1;

        // Current limit config
        public static final double CURRENT_LIMIT = 30.0;
        public static final double CURRENT_THRESHOLD = 50.0;
        public static final double TIMEOUT = 0.5;
    }

    public static final class ClimbConstants {
        public static final int LEFT_LIFT_MOTOR = 11;
        public static final int RIGHT_LIFT_MOTOR = 12;

        public static final int LEAD_ROTATE_MOTOR = 13;
        public static final int FOLLOW_ROTATE_MOTOR = 14;

        public static final double LIFT_MIN_POSITION = 0;
        public static final double LIFT_MAX_POSITION = 225559;

        // Pulse Width Locations
        public static final double ROTATE_MAX_ANGLE = 5000;
        public static final double ROTATE_MIN_ANGLE = 3400; 

        public static final double CLIMB_SPEED = 0.4;

        public static final double ROTATE_ENCODER_TICKS_PER_ROT = 1024.0;
        public static final double ROTATION_GEAR_RATIO = 80.0;

        public static final int CLIMB_LIMIT_SWITCH_PORT = 3;
    }

    public static final class IntakeConstants {
        public static final int DEPLOY_LEFT_MOTOR = 8;
        public static final int DEPLOY_RIGHT_MOTOR = 9;
        public static final int ROLLER_MOTOR = 10;

        public static final int RIGHT_LIMIT_SWITCH_UP = 3;
        public static final int LEFT_LIMIT_SWITCH_UP = 4;
        public static final int RIGHT_LIMIT_SWITCH_DOWN = 5;
        public static final int LEFT_LIMIT_SWITCH_DOWN = 6;

        public static final double MIN_INTAKE_ANGLE = 52.75;
        public static final double MAX_INTAKE_ANGLE = 15.0;

        public static final double INTAKE_P = 0.000005;

        public static final double DEPLOY_SPEED = 0.6;
        public static final double INTAKE_SPEED = 0.5;

        public static final int LIMIT_SWITCH_UP = 3;
        public static final int LIMIT_SWITCH_DOWN = 4;

        public static final int DEPLOY_CURRENT_LIMIT = 50;
        public static final int ROLLER_CURRENT_LIMIT = 30;
        public static final int CURRENT_DURATION = 500;

        public static final double ENCODER_TICKS_PER_ROTATION = 8192.0f;
    }

    public static final class ControllerConstants {
        // Joystick IDs
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;

        // Driver button IDs
        public static final int DRIVER_TOGGLE_LIMELIGHT_BTN = 1;
        public static final int DRIVER_FIELD_ORIENTED_BTN = 6;
        public static final int DRIVER_SLOW_MODE_BTN = 4;
        public static final int DRIVER_OUTTAKE_LOW_BTN = 8;
        public static final int DRIVER_OUTTAKE_HIGH_BTN = 3;

        // Operator button IDs
        public static final int OP_DEPLOY_INTAKE_BTN = XboxController.Button.kRightBumper.value;
        public static final int OP_OUTTAKE_HIGH_BTN = XboxController.Button.kLeftBumper.value;
        public static final int OP_CLIMB_MODE_BTN = XboxController.Button.kY.value;
        public static final int OP_ZERO_ROTATING_ARM_BTN = XboxController.Button.kStart.value;
        public static final int OP_REVERSE_OUTTAKE_BTN = XboxController.Button.kB.value;
        public static final int OP_TOGGLE_SLOW_BTN = XboxController.Button.kBack.value;
        public static final int OP_REVERSE_ALL_BTN = XboxController.Button.kX.value;
        public static final int OP_MANUAL_UPTAKE_BTN = XboxController.Button.kA.value;
    }
}
