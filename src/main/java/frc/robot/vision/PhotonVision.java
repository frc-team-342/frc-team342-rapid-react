// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;

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

    private double camHeight = Constants.VisionConstants.CAM_HEIGHT;
    private double targetHeight = Constants.VisionConstants.TARGET_HEIGHT;
    private double camAngle = Constants.VisionConstants.CAM_ANGLE;

    private String name;
    
    //Microsoft Camera
    private PhotonCamera msCam;

    public PhotonVision(String table) {
        msCam = new PhotonCamera(table);
        this.name = table;

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

    /**
     * Sets the driver mode 
     * 
     * @param mode True; Driver mode is selected False; Driver mode not selected
     */
    public void setDriverMode(boolean mode) {
        driverMode = mode;
        msCam.setDriverMode(driverMode);
    }

    /**
     * Sets driver mode to the opposite of its current mode
     */
    public void toggleDriverMode() {
        driverMode = !driverMode;
        setDriverMode(driverMode);
    }
    
    /**
     * Sets pipeline mode
     * @param mode {@link PipelineMode}
     */
    public void setPipeline(PipelineMode mode) {
        this.pipeline = mode;
        msCam.setPipelineIndex(pipeline.pipelineIndex);
    }

    /**
     * Gets current pipeline mode
     * @return Pipeline mode
     */
    public PipelineMode getPipeline() {
        return pipeline;
    }

    /**
     * Checks to see if photonvision has any targets
     * @return True if targets detected; False if no targets detected
     */
    public boolean hasTargets() {
        PhotonPipelineResult result = msCam.getLatestResult();
        return result.hasTargets();
    }

    /**
     * Gets horizontal offset from crosshair to target. 
     * @return Angle measure of offset
     */
    public double getHorizontalOffset() {
        PhotonPipelineResult result = msCam.getLatestResult();
        
        if(result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            return target.getYaw();
        }

       return Double.NaN;
    }

    /**
     * Gets vertical offset from crosshair to target.
     * @return Angle measure of offset 
     */
    public double getVerticalOffset() {
        PhotonPipelineResult result = msCam.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            return target.getPitch();   
        }

        return Double.NaN;
    }

    /**
     * Represents movement needed to get to target
     * @return Trajectory
     */
    public Transform2d transformToTarget() {
        PhotonPipelineResult result = msCam.getLatestResult();
        
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            return target.getCameraToTarget();
        }
        
        return null;
    }
    
    public double getDistance() {
        PhotonPipelineResult result = msCam.getLatestResult();

        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                camHeight,
                targetHeight, 
                camAngle,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        return -1;
    }
    
    public Map<String, Boolean> test() {
        Map<String, Boolean> results = new HashMap<>();

        var table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(name);

        // Check if latency value is present bc it should never be zero
        var entry = table.getEntry("latencyMillis").getNumber(0);
        results.put("Photon Camera", entry.intValue() != 0);

        return results;
    }
}