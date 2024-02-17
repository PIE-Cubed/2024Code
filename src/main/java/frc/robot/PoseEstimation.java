// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.Timer;

/**
 * Start of the PoseEstimation class
 */
public class PoseEstimation {
    // Object creation
    private Drive drive;
    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator visionEstimator;

    /**
     * The constructor for the PoseEstimation class
     * 
     * @param drive
     */
    public PoseEstimation(Drive drive) {
        // Instance creation
        this.drive = drive;
 
        // Starting module positions
        SwerveModulePosition[] moduleStartPosition = getAllModulePositions();

        // Creates the odometry tracker
        odometry = new SwerveDriveOdometry(
            drive.swerveDriveKinematics,
            new Rotation2d( drive.getYawAdjusted() ),
            moduleStartPosition,
            new Pose2d(0, 0, new Rotation2d(0))
        );

        // Defines the vision pose estimator's trust values (greater means less trusted)
        Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1));
        Vector<N3> visionStdDevs = VecBuilder.fill(0.9, 0.9, 1000000);

        // Creates the vision estimator
        visionEstimator = new SwerveDrivePoseEstimator(
            drive.swerveDriveKinematics,
            new Rotation2d( drive.getYawAdjusted() ),
            moduleStartPosition,
            new Pose2d(0, 0, new Rotation2d(-Math.PI)),
            stateStdDevs,
            visionStdDevs
        );
    }

    /****************************************************************************************** 
    *
    *    UPDATE FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Updates both Pose Estimators.
     */
    public void updatePoseTrackers() {
        updateOdometry();
        updateVisionEstimator();
    }

    /**
     * Updates the SwerveDriveOdometry.
     */
    public void updateOdometry() {
        // Compiles all the module positions
        SwerveModulePosition[] allModulePosition = getAllModulePositions();

        // Updates odometry
        odometry.update(
            new Rotation2d( MathUtil.angleModulus(drive.getYawAdjusted()) ),
            allModulePosition
        );
    }

    /**
     * Updates the VisionEstimator.
     */
    public void updateVisionEstimator() {
        // Compiles all the module positions
        SwerveModulePosition[] allModulePosition = getAllModulePositions();

        // Get the results from the limelight
        var results = LimelightHelpers.getLatestResults("limelight").targetingResults;

        // Gets the position from the results
        Pose2d bluePos = results.getBotPose2d_wpiBlue();

        // Calculates the latency from the results
        var timestamp = Timer.getFPGATimestamp() - (results.latency_capture / 1000) - (results.latency_pipeline / 1000);

        // Checks if there is a valid entry in the measurements
        if (results.targets_Fiducials.length > 0) {
            // Updates with vision if there is
            visionEstimator.addVisionMeasurement(bluePos, timestamp);
        }

        // Updates with odometry
        visionEstimator.update(
            new Rotation2d( MathUtil.angleModulus(drive.getYawAdjusted()) ),
            allModulePosition
        );
    }


    /****************************************************************************************** 
    *
    *    RESETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Resets both Pose Estimators to a defined position and rotation.
     * 
     * @param pose
     */
    public void resetPoseTrackers(Pose2d pose) {
        resetOdometry(pose);
        resetVisionEstimator(pose);
    }

    /**
     * Resets the SwerveDriveOdometry to a defined position and rotation.
     * 
     * @param pose
     */
    private void resetOdometry(Pose2d pose) {
        // Gets the module positions
        SwerveModulePosition[] allPositiions = getAllModulePositions();

        // Resets the SwerveOdometry
        odometry.resetPosition(
            new Rotation2d(MathUtil.angleModulus(drive.getYawAdjusted())),
            allPositiions,
            pose
        );
    }

    /**
     * Resets the VisionEstimator to a defined position and rotation.
     * 
     * @param pose
     */
    private void resetVisionEstimator(Pose2d pose) {
        // Gets the module positions
        SwerveModulePosition[] allPositiions = getAllModulePositions();

        // Resets the SwerveOdometry
        visionEstimator.resetPosition(
            pose.getRotation(),
            allPositiions,
            pose
        );
    }


    /****************************************************************************************** 
    *
    *    GETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Gets the floorPose as calulated by the SwerveDriveOdometry.
     * 
     * @return The robot's floor pose
     */
    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Gets the floorPose as calulated by the VisionEstimator.
     * 
     * @return The robot's floor pose
     */
    public Pose2d getVisionPose() {
        return visionEstimator.getEstimatedPosition();
    }

    /****************************************************************************************** 
    *
    *    HELPER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Gets the position of all four SwerveModules.
     * 
     * @return The position of all four wheels
     */
    private SwerveModulePosition[] getAllModulePositions() {
        // Creates a SwerveModlePosition array with all the wheels in it
        SwerveModulePosition[] allPositions = {
            drive.getFLPosition(),
            drive.getFRPosition(),
            drive.getBLPosition(),
            drive.getBRPosition()
        };

        return allPositions;
    }
}

// End of the PoseEstimation class