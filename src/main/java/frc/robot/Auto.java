// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Auto {
    // State tracking variables - each variable can only be used in one function at any time
    // All top level routines use firstTime and step, all helper routines have their own variables
    private int step;
    private boolean firstTime = true;

    private long delayEnd = 0; // Stores when delay() should return Robot.DONE
    private boolean delayFirstTime = true;

    // Object Creation
    private Drive          drive;
    private PoseEstimation position;
    private CustomTables   nTables;

    // Constructor
    public Auto(Drive drive, PoseEstimation position) {
        this.drive = drive;
        this.position = position;
        this.nTables  = CustomTables.getInstance();
    }

    public int testAuto(long delaySeconds) {
        int    status   = Robot.CONT;
        Pose2d currPose = position.getOdometryPose();

        if (firstTime == true) {
			firstTime = false;
			step = 1;
            System.out.println("Starting test auto");
		}

        switch(step) {
            case 1:
                // Delay
                status = autoDelay(delaySeconds);
                drive.rotateWheels(1, 0, 0);  
                break;
            
            case 2:
                status = drive.rotateWheels(1, 0, 0);
                break;

            case 3:
                Pose2d points = new Pose2d(1, 0, new Rotation2d(0));
                status = drive.autoDriveToPoints(new Pose2d[]{points}, currPose);
                break;

            default:
                // Finished routine
                step = 1;
                firstTime = true;

                // Stops applicable motors
                drive.stopWheels();
                return Robot.DONE;
        }

        // If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }


    /****************************************************************************************** 
    *
    *    HELPER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Delays the program for a set number of seconds.
     * 
     * @param seconds
     * @return status
     */
    public int autoDelay(long seconds) {
        long currentMS = System.currentTimeMillis();

        if (delayFirstTime) {
            delayEnd = currentMS + (seconds * 1000);
            delayFirstTime = false;
        }

        if (currentMS > delayEnd) {
            delayFirstTime = true;
            return Robot.DONE;
        }
        return Robot.CONT;
    }

    /**
     * Delays the program for a set number of milliseconds.
     * 
     * @param seconds
     * @return status
     */
    public int autoDelayMS(long ms) {
        long currentMS = System.currentTimeMillis();

        if (delayFirstTime) {
            delayEnd = currentMS + ms;
            delayFirstTime = false;
        }

        if (currentMS > delayEnd) {
            delayFirstTime = true;
            return Robot.DONE;
        }
        return Robot.CONT;
    }


    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/    
}
// End of the Auto class
