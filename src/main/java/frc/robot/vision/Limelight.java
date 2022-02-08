// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Limelight implements Sendable{

    private NetworkTable table;
    private NetworkTableEntry targets;
    private NetworkTableEntry horizontalOffset;
    private NetworkTableEntry verticalOffset;
    private NetworkTableEntry targetArea;
    private NetworkTableEntry camMode;
    private NetworkTableEntry robotPosition3D;

    double camAngle = Constants.VisionConstants.CAM_ANGLE;
    double targetHeight = Constants.VisionConstants.TARGET_HEIGHT;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        targets = table.getEntry("tv");
        horizontalOffset = table.getEntry("tx");
        verticalOffset = table.getEntry("ty");
        targetArea = table.getEntry("ta");
        camMode = table.getEntry("camMode");
        robotPosition3D = table.getEntry("cam-tran");
    }

    /**
     * Check whether any targets are currently detected
     * 
     * @return true if a target is found, false otherwise
     */
    public boolean hasTargets() {
        return targets.getBoolean(false);
    }
    
    /**
     * Gets horizontal offset from crosshair to target. 
     * 
     * @return -29.8 to 29.8 degrees
     */
    public double getHorizontalOffset() {
        return horizontalOffset.getDouble(0.0);
    }

    /**
     * Gets vertical offset from crosshair to target.
     * @return -24.85 to 24.85
     */
    public double getVerticalOffset() {
        return verticalOffset.getDouble(0.0);
    }

    /**
     * Gets the area of the target in the image
     * @return 0% of image to 100% of image 
     */
    public double getTargetArea() {
        return targetArea.getDouble(0.0);
    }

    /**
     * Limelight operation mode
     * @return 0 for vision, 1 for driver camera
     */
    public int getCamMode() {
        return camMode.getNumber(1).intValue();
    }

    /**
     * Sets limelight operation mode
     * @param mode 0 for vision, 1 for driver camera
     */
    public void setCamMode(int mode) {
        camMode.setNumber(mode);

    
    }
    
    /**
     * Looks to see if the crosshair is on the leftside of the target
     * @return True or False, if the crosshair is looking left
     */
    public boolean isLookingLeft() {
        return getHorizontalOffset() < 0;
    }

    /**
     * Gives us access to the position of the robot in a 3D field environment
     * @return The position of the robot with Translation (X,Y,Z) and Rotation (Pitch, Yaw, Roll) through a network table entry
     */
    public NetworkTableEntry getRobotPosition3D()
    {
        return robotPosition3D;
    }


    /**
     * Gives limelight access to a transform2d
     * Creates variables for the all the "cam-tran" network table entry values
     * Creates a translation2d and rotation2d for the limelight to use
     * Uses the translation2d and rotation2d to create a transform2d
     * @return The limelight instance of a transform2d
     */
    public Transform2d generateTransform()
    {
        //Gets a network table entry containing the X,Y,Z, pitch, yaw, and roll values from the limelight network table
        NetworkTableEntry robotPosition = getRobotPosition3D();

        //Gets the X-value from the "cam-tran" network table entry
        double robotPositionX = robotPosition.getDouble(0.0f);
        
        //Gets the Y-value from the "cam-tran" network table entry
        double robotPositionY = robotPosition.getDouble(0.0f);

        //Gets the Z-value from the "cam-tran" network table entry
        double robotPositionZ = robotPosition.getDouble(0.0f);

        //Gets the Pitch value from the "cam-tran" network table entry
        double robotRotationPitch = robotPosition.getDouble(0.0f);
        double robotRotationPitchRadians = Math.toRadians(robotRotationPitch);

        //Gets the Yaw value from the "cam-tran" network table entry
        double robotRotationYaw = robotPosition.getDouble(0.0f);
        double robotRotationYawRadians = Math.toRadians(robotRotationYaw);

        //Gets the Roll value from the "cam-tran" network table entry
        double robotRotationRoll = robotPosition.getDouble(0.0f);

        

        //Creates a Translation2d and a Rotation2d for use in a Transform2d value
        Translation2d limelightTranslation2d = new Translation2d(robotPositionX, robotPositionY);
        Rotation2d limelightRotation2d = new Rotation2d(robotRotationPitchRadians, robotRotationYawRadians);

        //Creates a transform2d for use by the limelight
        Transform2d limelightTransform2d = new Transform2d(limelightTranslation2d, limelightRotation2d);

        //Returns the limelight instance of Transform2d
        return limelightTransform2d;
    }

    /**
     * First finds the actual angle based on angling of the camera and vertical offset
     * Then finds distance from camera to target using trigonometry 
     * @return Horizontal distance converted to meters
     */
    public double getDistance() {
 
        double actAngle = getVerticalOffset() + camAngle;
        double hDistance = Units.inchesToMeters(targetHeight / (Math.tan(Math.toRadians(actAngle))));
        
        return hDistance;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Limelight");
        builder.addBooleanProperty("Has Targets", this::hasTargets, null);
        builder.addDoubleProperty("Horizontal Offset", this::getHorizontalOffset, null);
        builder.addDoubleProperty("Vertical Offset", this::getVerticalOffset, null);
        builder.addDoubleProperty("Cam Mode", this::getCamMode, null);
    }
}
