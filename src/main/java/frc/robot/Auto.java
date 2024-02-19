// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    private Arm            arm;
    private CustomTables   nTables;

    // Constructor
    public Auto(Drive drive, PoseEstimation position, Arm arm) {
        this.drive    = drive;
        this.position = position;
        this.arm      = arm;
        this.nTables  = CustomTables.getInstance();
    }

    /****************************************************************************************** 
     * 
     *    DRIVE FUNCTIONS
     * 
     ******************************************************************************************/    

    /**
     * Shoot the trap? need to confirm
     * 
     * @return Robot status, CONT or DONE
     */
    public int trapSequence() { return Robot.CONT; }

    /****************************************************************************************** 
     * 
     *    MANIPULATOR FUNCTIONS
     * 
     ******************************************************************************************/
    
    /**
     * <p>Moves the arm back into position to pick up a note from the ground
     * <p> -33.5 degrees from horizontal
     * <p> 5inch extension of the arm
     * <p> First rotates, then extends the arm
     * @return Robot status, CONT or DONE
     */
    public int groundPickupPosition() {
        int status = Robot.CONT;

        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {
            case 1:     // Rotate arm to -33.5 degrees from horizontal
                status = arm.rotateArm(Math.toDegrees(-33.5));
                break;
            case 2:     // Extend arm to 5in(current function takes encoder rotations right now)
                status = arm.extendArm(1);  // TODO get actual distance
                break;
            default:    // Finished routine, reset variables and return done
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * <p> Moves the arm back into resting position
     * <p> -27 degrees from horizontal
     * <p> Fully retracts arm
     * <p> First rotates, then retracts the arm
     * @return Robot status, CONT or DONE
     */
    public int restingPosition() {
        int status = Robot.CONT;

        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {
            case 1:     // Rotate arm to -27 degrees from horizontal
                status = arm.rotateArm(Math.toDegrees(-27));
                break;
            case 2:     // Fully retract arm
                status = arm.extendArm(0);
                break;
            default:    // Finished routine, reset variables and return done
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * Moves the arm into the position to shoot into the amp
     * TODO get arm angle and extension measurments from austin
     */
    public int ampPosition() {
        System.err.println("Not implemented yet!");

        int status = Robot.CONT;

        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {
            case 1:     // Rotate arm to position(need to get this angle)
                break;
            case 2:     // Fully retract arm(need to confirm with austin)
                break;
            default:    // Finished routine, reset variables and return done
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * Sets up and moves the arm to climb the chain
     * 
     * @return Robot.CONT or Robot.DONE
     */
    public int autoClimb() {
        System.err.println("Not implemented yet!");

        int status = Robot.CONT;

        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {
            case 1:     // Rotate arm vertically
                break;
            case 2:     // Extend arm to mount chain
                break;
            case 3:     // Retract arm fully to climb 
                break;
            default:    // Finished routine, reset variables and return done
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
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
