// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class DriveConstants {
        // Motor IDs
        public static final int FRONT_LEFT_MOTOR = 1;
        public static final int BACK_LEFT_MOTOR = 2;
        public static final int FRONT_RIGHT_MOTOR = 3;
        public static final int BACK_RIGHT_MOTOR = 4;

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
}
