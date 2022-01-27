// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class PhotonVision implements Sendable {

    public enum PipelineMode {
        BLUE(5), 
        RED(6), 
        NONE(1);

        public final int pipelineIndex;
        private PipelineMode(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }
    }

    private boolean driverMode = true;
    private PipelineMode pipeline;

    //Microsoft Camera
    private PhotonCamera msCam;

    public PhotonVision(String table) {
        msCam = new PhotonCamera(table);

        // Initialize the pipeline mode depending on which alliance
        boolean redAlliance = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
        if (redAlliance) {
            setPipeline(PipelineMode.RED);
        } else {
            setPipeline(PipelineMode.BLUE);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PhotonVision");
        builder.addBooleanProperty("Driver Mode", this::getDriverMode, null);
    }

    /**
     * Gets the current state of the camera's Driver Mode
     * @return
     */
    public boolean getDriverMode() {
        return driverMode;
    }

    public void toggleDriverMode() {
        driverMode = !driverMode;
    }
    
    public void setPipeline(PipelineMode mode) {
        this.pipeline = mode;
        msCam.setPipelineIndex(pipeline.pipelineIndex);
    }

    public PipelineMode getPipeline() {
        return pipeline;
    }
}