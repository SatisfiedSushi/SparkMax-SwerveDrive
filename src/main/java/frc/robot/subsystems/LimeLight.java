// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class LimeLight extends SubsystemBase {
    // NetworkTable fields
    NetworkTable table;
    NetworkTableEntry tv;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tid;
    NetworkTableEntry getpipe;
    NetworkTableEntry pipeline;

    // Other fields
    XboxController m_controller;

    /**
     * Creates a new LimeLight subsystem.
     */
    public LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tid = table.getEntry("tid");
        getpipe = table.getEntry("getpipe");
        pipeline = table.getEntry("pipeline");
    }

    /**
     * Returns the x angle of the detected object from the limelight's heading in
     * degrees.
     * 
     * @return The x angle of the detected object or 0.0 if no object is found.
     */
    public double getXAngle() {
        return tx.getDouble(0.0);
    }

    /**
     * Returns the y angle of the detected object from the limelight's heading in
     * degrees.
     * 
     * @return The y angle of the detected object or 0.0 if no object is found.
     */
    public double getYAngle() {
        return ty.getDouble(0.0);
    }

    /**
     * Returns the area percent of the detected object.
     * 
     * @return The area percent of the detected object or 0.0 if no object is found.
     */
    public double getArea() {
        return ta.getDouble(0.0);
    }

    /**
     * Returns the current targeted AprilTag ID.
     * 
     * @return The current targeted AprilTag ID or 0.0 if no object is found.
     */
    public double getTagID() {
        return tid.getDouble(0.0);
    }

    /**
     * Returns the current pipeline number (0-9).
     * 
     * @return The current pipeline number or 0.0 if no object is found.
     */
    public double getPipline() {
        return getpipe.getDouble(0.0);
    }

    /**
     * Returns if the limelight has a valid target (tv=1).
     * 
     * @return True if the limelight has a valid target, false otherwise.
     */
    public boolean hasTarget() {
        if (tv.getDouble(0.0) == 1) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Set pipline number between 0-9.
     * 
     * @param pipelineNumber The pipeline number to set.
     */
    public void setPipeline(double pipelineNumber) {
        pipeline.setValue(pipelineNumber);
    }

    /**
     * Set pipline number between 0-9.
     * 
     * @param targetedAprilTagId The targeted AprilTag ID to set
     */
    public void setTargetedAprilTagId(double targetedAprilTagId) {
        setPipeline(targetedAprilTagId + 1);
    }

    /**
     * Taken from PhotonVision's open source code:
     * https://github.com/PhotonVision/photonvision/blob/master/photon-lib/src/main/java/org/photonvision/PhotonUtils.java
     *
     * <p>
     * Units can be converted using the {@link edu.wpi.first.math.util.Units} class.
     *
     * @param cameraHeightMeters The physical height of the camera off the floor in
     *                           meters.
     * @param targetHeightMeters The physical height of the target off the floor in
     *                           meters. This
     *                           should be the height of whatever is being targeted
     *                           (i.e. if the targeting region is set to
     *                           top, this should be the height of the top of the
     *                           target).
     * @param cameraPitchRadians The pitch of the camera from the horizontal plane
     *                           in radians.
     *                           Positive values up.
     * @param targetPitchRadians The pitch of the target in the camera's lens in
     *                           radians. Positive
     *                           values up.
     * @return The estimated distance to the target in meters.
     */
    public double calculateDistanceToTargetMeters(double cameraHeightMeters, double targetHeightMeters,
            double cameraPitchRadians, double targetPitchRadians) {
        return (targetHeightMeters - cameraHeightMeters) / Math.tan(cameraPitchRadians + targetPitchRadians);
    }

    /**
     * Deprecated method. Use
     * {@link #calculateDistanceToTargetMeters(double, double, double, double)}
     * instead.
     * 
     * @param targetHeight
     * @return distance in inches from limelight to reflective tape target or -1 if
     *         no valid targets are seen by Limelight camera (i.e., tv=0).
     */
    public double calculateDistance(double targetHeight) {

        double distance = 0;

        // with area (doesnt fucking work im ass mb)
        /*
         * double area = getArea();
         * if (area < 1) {
         * distance = -1; //-1 means the limelight does not see a target
         * } else { //calculate the distance based on the area of the target and some
         * constants that we found experimentally. This is not perfect, but it works for
         * our robot.
         * distance = (0.0028 * Math.pow(area, 2)) + (-0.5 * area) + 50; //distance in
         * inches from limelight to target based on ta value and a curve fit of
         * experimental data points taken by measuring actual distances vs values of ta
         * reported by Limelight for those distances with reflective tape as a target at
         * different angles to camera lens axis
         * 
         * SmartDashboard.putNumber("Distance", distance);
         * 
         * return distance; //returns calculated distance in inches from limelight to
         * reflective tape target or -1 if no valid targets are seen by Limelight camera
         * (i.e., tv=0)
         * 
         * }
         * 
         * return 0;
         */

        // Using known variables (angles, heights, etc.)
        // From limelight documentation
        /*
         * if (hasTarget()) {
         * distance = -1; //-1 means the limelight does not see a target
         * } else { //calculate the distance based on the area of the target and some
         * constants that we found experimentally. This is not perfect, but it works for
         * our robot.
         * double angleToGoalDegrees = Constants.LIMELIGHT_MOUNT_ANGLE + getYAngle();
         * double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
         * 
         * distance = (Constants.LIMELIGHT_GOAL_HEIGHT -
         * Constants.LIMELIGHT_LENS_HEIGHT)/Math.tan(angleToGoalRadians);
         * }
         */

        // Distance calculation from
        // https://www.chiefdelphi.com/t/calculating-distance-to-vision-target/387183/6
        if (hasTarget()) {
            distance = 0; // -1 means the limelight does not see a target
        } else {
            double z = 1 / (Math.sqrt(1 + Math.pow(Math.tan(getYAngle()), 2) + Math.pow(Math.tan(getXAngle()), 2)));
            double y = getYAngle()
                    / (Math.sqrt(1 + Math.pow(Math.tan(getYAngle()), 2) + Math.pow(Math.tan(getXAngle()), 2)));
            double x = getXAngle()
                    / (Math.sqrt(1 + Math.pow(Math.tan(getYAngle()), 2) + Math.pow(Math.tan(getXAngle()), 2)));

            double scaleFactor = (targetHeight - LimelightConstants.kLLHeight) / y;

            distance = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2)) * scaleFactor;
        }

        // 20.32 accounts for the distance from the limelight to the center of the robot
        return distance; // returns calculated distance in inches from limelight to reflective tape
                         // target or 0 if no valid targets are seen by Limelight camera (i.e., tv=0)
    }

    /**
     * Updates SmartDashboard.
     */
    public void updateDashboard() {
        SmartDashboard.putNumber("LimelightX", getXAngle());
        SmartDashboard.putNumber("LimelightY", getYAngle());
        SmartDashboard.putNumber("LimelightArea", getArea());
        SmartDashboard.putNumber("CurrentTargetedTagID", getTagID());
        SmartDashboard.putNumber("CurrentPipline", getPipline());
        // SmartDashboard.putNumber("Object Distance", calculateDistanceToTargetMeters(,
        // 0.63, , 0)); //mm
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler
        updateDashboard();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
