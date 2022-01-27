// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class PhotonVision implements Sendable {

    private boolean driverMode = true;
    private int pipelineIndex = 0;

    //Microsoft Camera
    PhotonCamera msCam;

    public PhotonVision(String table) {
        msCam = new PhotonCamera(table);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PhotonVision");
        builder.addBooleanProperty("Driver Mode", this::getDriverMode, null);
        builder.addStringProperty("Pipeline Index", this::getPipeline, this::setPipeline);
    }
    /**
     * Gets the current state of the camera's Driver Mode
     * @return
     */
    public boolean getDriverMode() {
        return driverMode;
    }

    /**
     * Sets the pipeline of the Microsoft Camera based on the string inputs given in the Shuffleboard
     * @param color color is what color team we are on during the match
     */
    public void setPipeline(String color) {
        String team = color.toLowerCase();
        if (team.equals("blue")) {
            msCam.setPipelineIndex(5);
            pipelineIndex = 5;
        }
        else if (team.equals("red")) {
            msCam.setPipelineIndex(6);
            pipelineIndex = 6;
        }
        else {
            msCam.setPipelineIndex(1);
            pipelineIndex = 1;
        }
    }

    /**
     * Gets the current state of the Pipeline of the Microsoft Camera and returns which color cargo we are targeting
     * @return
     */
    public String getPipeline() {
        if (pipelineIndex == 5) {
            return "Blue";
        }
        else if (pipelineIndex == 6) {
            return "Red";
        }
        else {
            return "null";
        }
    }
}