// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public class Auto {
    // State tracking variables - each variable can only be used in one function at any time
    // All top level routines use firstTime and step, all helper routines have their own variables
    private int step;
    private boolean firstTime = true;
    private boolean teleopShootFirstTime = true;

    private long delayEnd = 0; // Stores when delay() should return Robot.DONE
    private boolean delayFirstTime = true;

    
    private int intakeStatus = Robot.CONT;
    private int driveStatus = Robot.CONT;
    private int status = Robot.CONT;

    // Auto program selection
    public String selectedAuto = "Speaker Center";

    // Object Creation
    private Drive          drive;
    private PoseEstimation position;
    private Arm            arm;
    private CustomTables   nTables;
    private Grabber        grabber;
    private Shooter        shooter;

    // Constructor
    public Auto(Drive drive, PoseEstimation position, Arm arm, Grabber grabber, Shooter shooter) {
        this.drive    = drive;
        this.grabber  = grabber;
        this.position = position;
        this.arm      = arm;
        this.nTables  = CustomTables.getInstance();
        this.shooter  = shooter;
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
            // Rotate arm to -33.5 degrees from horizontal
            case 1:
                status = arm.rotateArm(Math.toDegrees(-33.5));
                break;

            // Extend arm to 5in(current function takes encoder rotations right now)
            case 2:
                status = arm.extendArm(1, 0.1);  // TODO get actual distance
                break;

            // Finished routine, reset variables and return done
            default:
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, go to next one
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
            case 1:     // Rotate arm to -27 degrees from horizontal (359 - 27 = 332)
                status = arm.rotateArm(332);
                break;
            case 2:     // Fully retract arm
                status = arm.extendArm(0, 0.1);
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
     * <p> Moves the arm so the shooter is aiming at the speaker
     * <p> -27 degrees from horizontal
     * <p> Fully retracts arm
     * <p> First rotates, then retracts the arm
     * @return Robot status, CONT or DONE
     */
    public int speakerPosition() {
        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {            
            // Start the shooter motors and Let them spin up for 2 seconds
            case 1:
                shooter.spinup();
                status = autoDelay(2);
                break;

            // Rotate the arm to -23 (336) degrees from 54
            case 2:
                status = arm.rotateArm(333);
                break;
            
            // Extend the arm so the wood holding block falls into the robot, and so the arm is in the shooting position
            case 3:
                status = arm.extendArm(14, 0.25);
                arm.maintainPosition(333);
                break;
                        
            // Shoot the note by running the grabber
            case 4:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(333);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 1 second(s)
            case 5:
                status = autoDelay(1);
                arm.maintainPosition(333);
                break;

            // Rotate the arm to it's resting position and turn off the shooter and Switch the grabber to intake mode
            case 6:
                status = arm.rotateArm(322);
                break;

            // Drive backwards 5 feet
            case 7:
                if (intakeStatus == Robot.CONT) {
                    intakeStatus = grabber.intakeOutake(true, false);
                }
                
                if (driveStatus == Robot.CONT) {
                    driveStatus = drive.driveDistanceWithAngle(0, 4, 0.3);
                }
                else {
                    System.out.println("Robot has finished driving.");
                }
                
                if (intakeStatus == Robot.DONE && driveStatus == Robot.DONE) {
                    drive.resetDriveDistanceFirstTime();
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
            
                break;

            // Drive back to the speaker
            case 8:
                status = drive.driveDistanceWithAngle(0, -3, 0.3);
                break;

            // Rotate the arm so it's in the shooting position
            case 9:
                status = arm.rotateArm(338); // Use 343 if not driving to speaker          
                break;

            // Shoot the note
            case 10:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(335);
                status = autoDelay(2);
                break;
            
            // Finished routine, reset variables and return done
            default:
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
     * <p> Moves the arm so the shooter is aiming at the speaker
     * <p> -27 degrees from horizontal
     * <p> Fully retracts arm
     * <p> First rotates, then retracts the arm
     * @return Robot status, CONT or DONE
     */
    public void teleopShoot() {
        int intakeStatus = Robot.CONT;
        int driveStatus = Robot.CONT;
        int status = Robot.CONT;

        if(teleopShootFirstTime == true) {
            teleopShootFirstTime = false;
            step = 1;
        }

        switch(step) {            
            // Start the shooter motors and rotate the arm to -23 (336) degrees from 54
            case 1:
                shooter.spinup();
                status = arm.rotateArm(333);
                break;

            // Wait for the thooter motors to spin up
            case 2:                
                arm.maintainPosition(333);
                status = autoDelay(1);
                break;
            
            // Start the grabber and keep the arm in shooting position
            default:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(333);
                break;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }
    }

    /*
     * Set Teleop shooting first time to true
     */
    public void resetTeleopShoot() {
        teleopShootFirstTime = true;
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
     * <p> Moves the arm to its starting position
     * <p> 54 degrees from horizontal
     * <p> Fully retracts arm
     * <p> First rotates, then extends the arm
     * @return Robot status, CONT or DONE
     */
    public int startingPosition() {
        int status = Robot.CONT;

        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {
            case 1:     // Rotate arm to position (54 degrees)
                status = arm.rotateArm(54);
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
