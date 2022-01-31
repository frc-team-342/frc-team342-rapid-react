// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Limelight implements Sendable{

    private NetworkTable table;
    private NetworkTableEntry targets;
    private NetworkTableEntry horizontalOffset;
    private NetworkTableEntry verticalOffset;
    private NetworkTableEntry targetArea;
    private NetworkTableEntry camMode;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        targets = table.getEntry("tv");
        horizontalOffset = table.getEntry("tx");
        verticalOffset = table.getEntry("ty");
        targetArea = table.getEntry("ta");
        camMode = table.getEntry("camMode");
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

    public boolean isLookingLeft() {
        return getHorizontalOffset() < 0;
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
