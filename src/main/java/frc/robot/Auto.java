// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.Robot.ArmState;

public class Auto {
    // State tracking variables - each variable can only be used in one function at any time
    // All top level routines use firstTime and step, all helper routines have their own variables
    private int step;
    private boolean firstTime = true;
    private boolean teleopShootFirstTime = true;
    public boolean isRed = false;
    public int allianceAngleModifier = 1;

    private long delayEnd = 0; // Stores when delay() should return Robot.DONE
    private boolean delayFirstTime = true;

    
    private int intakeStatus = Robot.CONT;
    private int driveStatus = Robot.CONT;
    private int armStatus = Robot.CONT;
    private int status = Robot.CONT;
    
    private final int SHOOT1_ANGLE = 339;   // Up against speaker
    //private final int SHOOT1_ANGLE = 349;   // Edge of safe zone
    private final int SHOOT2_ANGLE = 339;

    // Auto program selection
    //public String selectedAuto = "Speaker Center";

    // Object Creation
    private Drive          drive;
    //private PoseEstimation position;
    private CustomTables   nTables;
    private Arm            arm;
    private Grabber        grabber;
    private Shooter        shooter;

    // Constructor
    public Auto(Drive drive, PoseEstimation position, Arm arm, Grabber grabber, Shooter shooter) {
        this.drive    = drive;
        this.grabber  = grabber;
        //this.position = position;
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
     * <p> Doesnt retract arm(loses precision from string slack)
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
                status = arm.rotateArm(350);
                break;
            case 2:
                arm.maintainPosition(350);
                status = arm.extendArm(0, -0.1);
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
     * <p> Moves the arm so the shooter is aiming at the speaker (located in the center position)
     * <p> -26 degrees from horizontal
     * <p> Fully retracts arm
     * <p> First rotates, then retracts the arm
     * <p> This currently takes 11 seconds for a full auto cycle
     * @return Robot status, CONT or DONE
     */
    public int extensionTest() {
        if(firstTime == true) {
            firstTime = false;
            intakeStatus = Robot.CONT;
            driveStatus = Robot.CONT;
            step = 1;
        }

        switch(step) {
            // Extend the arm so the wood holding block falls into the robot
            case 1:
            System.out.println("RETRACT");
            //System.out.println("Arm extension position: " + arm.getExtendPosition());
                status = arm.extendArm(23, 0.25);
                break;

            // Extend the arm so the wood holding block falls into the robot
            case 2:
            System.out.println("EXTEND");
            //System.out.println("Arm extension position: " + arm.getExtendPosition());
                status = arm.extendArm(-23, -0.25);
                break;
                
            // Finished routine, reset variables, stop motors, and return done
            default:
            System.out.println("DONE");
                shooter.stopShooting();
                grabber.intakeOutake(false, false);
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
     * <p> Moves the arm so the shooter is aiming at the speaker (located in the center position)
     * <p> -26 degrees from horizontal
     * <p> Fully retracts arm
     * <p> First rotates, then retracts the arm
     * <p> This currently takes 11 seconds for a full auto cycle
     * @return Robot status, CONT or DONE
     */
    public int speakerPositionCenter() {
        if(firstTime == true) {
            firstTime = false;
            intakeStatus = Robot.CONT;
            driveStatus = Robot.CONT;
            step = 1;
        }

        switch(step) {            
            // Rotate the drive motors to zero
            case 1:
                status = drive.rotateWheelsToAngle(0);
                break;

            // Retract the arm fully to prevent out of bounds issues
            case 2:
                status = arm.extendArm(23, 0.25);
                break;
            
            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            case 3:
                shooter.spinup();
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;

            // Extend the arm to its original position
            case 4:
                status = arm.extendArm(-16, -0.25);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;
                        
            // Shoot the note by running the grabber
            case 5:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 0.6 second(s)
            case 6:
                status = autoDelayMS(600);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;

            // Rotate the arm to it's resting position and turn off the shooter and Switch the grabber to intake mode
            case 7:
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;

            // Extend the arm so the wood holding block falls into the robot, and so the arm is in the shooting position
            case 8:
                status = Robot.DONE;
                // = arm.extendArm(8, 0.3);
                break;

            // Drive backwards 4 feet
            case 9:
                if (intakeStatus == Robot.CONT) {
                    intakeStatus = grabber.intakeOutake(true, false);
                }
                
                if (driveStatus == Robot.CONT) {
                    driveStatus = drive.driveDistanceWithAngle(0, 4.5, 0.2);
                }
                
                if (intakeStatus == Robot.DONE && driveStatus == Robot.DONE) {
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
            
                break;

            case 10:
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;

            // Drive back to the speaker
            case 11:
                arm.maintainPosition(SHOOT1_ANGLE);
                status = drive.driveDistanceWithAngle(0, -5.3, 0.5);
                break;

            // Rotate the arm so it's in the shooting position
            case 12:
                status = arm.rotateArm(SHOOT2_ANGLE);    
                break;

            // Shoot the note
            case 13:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT2_ANGLE);
                status = autoDelay(1);
                break;
            
            // Finished routine, reset variables, stop motors, and return done
            default:
                shooter.stopShooting();
                grabber.intakeOutake(false, false);
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
     * <p> Moves the arm so the shooter is aiming at the speaker (located in the right position)
     * <p> -26 degrees from horizontal
     * <p> Fully retracts arm
     * <p> First rotates, then retracts the arm
     * <p> This currently takes 11 seconds for a full auto cycle
     * @return Robot status, CONT or DONE
     */
    public int speakerPositionAmp() {
        if(firstTime == true) {
            firstTime = false;
            intakeStatus = Robot.CONT;
            driveStatus = Robot.CONT;
            step = 1;
        }

        switch(step) {   
            // Rotate the drive motors to zero
            case 1:
                status = drive.rotateWheelsToAngle(0);
                break;
                
            // Retract the arm fully to prevent out of bounds issues
            case 2:
                status = arm.extendArm(23, 0.25);
                break;
            
            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            case 3:
                shooter.spinup();
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;

            // Extend the arm to its original position
            case 4:
                status = arm.extendArm(-16, -0.25);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;
                        
            // Shoot the note by running the grabber
            case 5:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 0.75 second(s)
            case 6:
                status = autoDelayMS(750);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;

            // Rotate the arm to its resting position, and turn off the shooter & grabber
            case 7:            
                shooter.stopShooting();
                grabber.intakeOutake(false, false);
                status = arm.rotateArm(327);
                break;

            // Extend the arm so the wood block falls into the robot
            /*case 6:
                status = arm.extendArm(14, 0.2);
                break;*/

            // Drive forwards 1 foot
            case 8:
                status = drive.driveDistanceWithAngle(0, 1, 0.5);            
                break;

            // Rotate the robot 57 degrees
            case 9:
                status = drive.rotateRobot(Math.toRadians(allianceAngleModifier * 57));
                break;

            // Rotate the wheels back to zero before driving forward
            case 10:
                status = drive.rotateWheelsToAngle(0);            
                grabber.intakeOutake(true, false);
                break;

            // Drive forwards 4 feet and pick up a note
            case 11:
                if (intakeStatus == Robot.CONT) {
                    intakeStatus = grabber.intakeOutake(true, false);
                }
                
                if (driveStatus == Robot.CONT) {
                    driveStatus = drive.driveDistanceWithAngle(0, 4, 0.5);
                }
                
                if (intakeStatus == Robot.DONE && driveStatus == Robot.DONE) {
                    driveStatus = Robot.CONT;
                    intakeStatus = Robot.CONT;
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
            
                break;

            // Reset Wheel angle to 0 and raise arm to 333 degrees to avoid note dragging
            case 12:
                if(armStatus == Robot.CONT) {
                    armStatus = arm.rotateArm(333);
                }

                if(driveStatus == Robot.CONT) {
                    driveStatus = drive.rotateWheelsToAngle(0);
                }
                
                if (armStatus == Robot.DONE && driveStatus == Robot.DONE) {
                    driveStatus = Robot.CONT;
                    armStatus = Robot.CONT;
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }

                break;
                
            // Drive Backwards 4 feet
            case 13:
                arm.maintainPosition(333);
                status = drive.driveDistanceWithAngle(0, -4.75, 0.5);
                break;

            // Rotate the robot back to 0
            case 14:
                arm.maintainPosition(333);
                status = drive.rotateRobot(Math.toRadians(0));
                break;

            // Reset wheel angle to 0
            //case 14:
            //    arm.maintainPosition(333);
            //    status = drive.rotateWheelsToAngle(0);
            //    break;

            // Drive backwards 1 feet
            case 15:
                arm.maintainPosition(333);
                status = drive.driveDistanceWithAngle(0, -1.5, 0.5);
                break;
            /*// Rotate robot to 28 to face speaker directly
            case 11:
                arm.maintainPosition(333);
                status = drive.rotateRobot(Math.toRadians(28));
                break;

            case 12:
                status = drive.rotateWheelsToAngle(0);
                break;

            // Drive back to speaker 5.5ft
            case 13:
                arm.maintainPosition(333);
                status = drive.driveDistanceWithAngle(0, -5.5, 0.5);
                break;

            */
            // Shoot
            case 16:
                shooter.spinup();
                status = arm.rotateArm(SHOOT2_ANGLE);
                break;

            // Shoot the note
            case 17:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT2_ANGLE);
                status = autoDelay(1);
                break;

            // Finished routine, reset variables, stop motors, and return done
            default:
                shooter.stopShooting();
                grabber.intakeOutake(false, false);
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
     * <p> Shoots, then backs up to clear note and stage, drives out, then back
     * @return Robot status, CONT or DONE
     */
    public int speakerPositionFeed() {
        if(firstTime == true) {
            firstTime = false;
            intakeStatus = Robot.CONT;
            driveStatus = Robot.CONT;
            step = 1;
        }

        switch(step) {
            // Rotate the drive motors to zero
            case 1:
                status = drive.rotateWheelsToAngle(0);
                break;

            // Retract the arm fully to prevent out of bounds issues
            case 2:
                status = arm.extendArm(23, 0.25);
                break;
            
            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            case 3:
                shooter.spinup();
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;

            // Extend the arm to its original position
            case 4:
                status = arm.extendArm(-16, -0.25);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;
                        
            // Shoot the note by running the grabber
            case 5:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 0.75 second(s)
            case 6:
                status = autoDelayMS(750);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;

            // Rotate the arm to its resting position, and turn off the shooter & grabber
            case 7:            
                shooter.stopShooting();
                grabber.intakeOutake(false, false);
                status = arm.rotateArm(327);
                break;

            // Drive forwards 1 foot
            case 8:
                status = drive.driveDistanceWithAngle(0, 6, 0.5);            
                break;

            // Rotate the robot 57 degrees
            case 9:
                status = drive.rotateRobot(Math.toRadians(allianceAngleModifier * -57));
                break;

            // Rotate the wheels back to zero before driving forward
            case 10:
                status = drive.rotateWheelsToAngle(0);            
                break;
            
            // Drive forwards to get out of alliance area
            case 11:
                status = drive.driveDistanceWithAngle(0, 3, 0.5);
                break;

            // Drive backwards to get back into alliance area
            case 12:
                status = drive.driveDistanceWithAngle(0, -3.5, 0.5);
                break;

            // Finished routine, reset variables, stop motors, and return done
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
     * <p> Faces the speaker
     * <p> Fail stops the auto immediatly, resets, and returns fail
     * @return Robot status, CONT, DONE, or FAIL
     */
    public int targetSpeaker() {
        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {
            case 1:
                status = drive.alignWithAprilTag();
                break;
            case 2:
                status = drive.rotateWheelsToAngle(0);
                break;
            default:
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }
        else if(status == Robot.FAIL) { // Fail, reset and return fail
            System.out.println("No Valid AprilTag in Frame!!!");
            step = 1;
            firstTime = true;
            return Robot.FAIL;
        }

        return Robot.CONT;
    }

    /**
     * <p> Moves the arm so the shooter is aiming at the speaker (located in the center position)
     * <p> -26 degrees from horizontal
     * <p> Fully retracts arm
     * <p> First rotates, then retracts the arm
     * <p> This currently takes 11 seconds for a full auto cycle
     * @return Robot status, CONT or DONE
     */
    public int speakerShootWithoutMoving() {
        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {            
            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            case 1:
                shooter.spinup();
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;
                        
            // Shoot the note by running the grabber
            case 2:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 1 second(s)
            case 3:
                status = autoDelay(1);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;

            // Rotate the arm to it's resting position and turn off the shooter and Switch the grabber to intake mode
            case 4:
                status = arm.rotateArm(322);
                break;

            // Extend the arm so the wood holding block falls into the robot, and so the arm is in the shooting position
            case 5:
                status = arm.extendArm(8, 0.25);
                break;
            
            // Finished routine, reset variables, stop motors, and return done
            default:
                shooter.stopShooting();
                grabber.intakeOutake(false, false);
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
    public int teleopShoot(boolean shooterEnable) {
        //int intakeStatus = Robot.CONT;
        //int driveStatus = Robot.CONT;
        int status = Robot.CONT;

        if(teleopShootFirstTime == true) {
            teleopShootFirstTime = false;
            step = 1;
        }

        switch(step) {            
            // Start the shooter motors and rotate the arm to -23 (336) degrees from 54
            case 1:
                shooter.spinup();
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;
            
            // Start the grabber and keep the arm in shooting position
            case 2:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                
                if(shooterEnable) {
                    status = Robot.CONT;
                }
                else {
                    status = Robot.DONE;
                }
                break;
            
            default:
                teleopShootFirstTime = true;
                shooter.stopShooting();
                grabber.intakeOutake(false, false);
                arm.disableRotation();
                step = 1;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
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
        int status = Robot.CONT;

        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {
            case 1:     // Rotate arm to 90 degrees
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
