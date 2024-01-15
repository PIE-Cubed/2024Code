// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;

/**
 * Start of the PoseEstimation class
 */
public class PoseEstimation {
    // Object creation
    private Drive drive;
    private CustomTables nTables;
    private SwerveDriveOdometry odometry;

    /**
     * The constructor for the PoseEstimation class
     * 
     * @param drive
     */
    public PoseEstimation(Drive drive) {
        // Instance creation
        this.drive   = drive;
        this.nTables = CustomTables.getInstance();
 
        // Starting module positions
        SwerveModulePosition[] moduleStartPosition = getAllModulePositions();

        // Creates the odometry tracker
        odometry = new SwerveDriveOdometry(
            drive.swerveDriveKinematics,
            new Rotation2d( drive.getYawAdjusted() ),
            moduleStartPosition,
            new Pose2d(0, 0, new Rotation2d(-Math.PI))
        );
    }

    /****************************************************************************************** 
    *
    *    UPDATE FUNCTIONS
    * 
    ******************************************************************************************/
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


    /****************************************************************************************** 
    *
    *    RESETTER FUNCTIONS
    * 
    ******************************************************************************************/
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