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

    private final int SHOOT1_ANGLE = 331;
    private final int SHOOT2_ANGLE = 331;

    // Auto program selection
    //public String selectedAuto = "Speaker Center";

    // Object Creation
    private Drive          drive;
    //private PoseEstimation position;
    //private CustomTables   nTables;
    private Arm            arm;
    private Grabber        grabber;
    private Shooter        shooter;

    // Constructor
    public Auto(Drive drive, PoseEstimation position, Arm arm, Grabber grabber, Shooter shooter) {
        this.drive    = drive;
        this.grabber  = grabber;
        //this.position = position;
        this.arm      = arm;
        //this.nTables  = CustomTables.getInstance();
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
                status = arm.rotateArm(332);
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
            
            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            case 2:
                shooter.spinup();
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;
                        
            // Shoot the note by running the grabber
            case 3:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 0.75 second(s)
            case 4:
                status = autoDelayMS(750);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;

            // Rotate the arm to it's resting position and turn off the shooter and Switch the grabber to intake mode
            case 5:
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;

            // Extend the arm so the wood holding block falls into the robot, and so the arm is in the shooting position
            case 6:
                status = Robot.DONE;
                //status = arm.extendArm(8, 0.3);
                break;

            // Drive backwards 4 feet
            case 7:
                if (intakeStatus == Robot.CONT) {
                    intakeStatus = grabber.intakeOutake(true, false);
                }
                
                if (driveStatus == Robot.CONT) {
                    driveStatus = drive.driveDistanceWithAngle(0, 4, 0.5);
                }
                
                if (intakeStatus == Robot.DONE && driveStatus == Robot.DONE) {
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
            
                break;

            // Drive back to the speaker
            case 8:
                status = drive.driveDistanceWithAngle(0, -4.5, 0.5);
                break;

            // Rotate the arm so it's in the shooting position
            case 9:
                status = arm.rotateArm(SHOOT2_ANGLE); // Use 343 if not driving to speaker          
                break;

            // Shoot the note
            case 10:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT2_ANGLE);
                status = autoDelay(1);
                break;
            
            // Finished routine, reset variables, stop motors, and return done
            default:
                shooter.stopShooting();
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
    public int speakerPositionRight() {
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

            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            case 2:
                shooter.spinup();
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;
                        
            // Shoot the note by running the grabber
            case 3:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 0.75 second(s)
            case 4:
                status = autoDelayMS(750);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;

            // Rotate the arm to its resting position, and turn off the shooter & grabber
            case 5:            
                shooter.stopShooting();
                status = arm.rotateArm(327);
                break;

            // Extend the arm so the wood block falls into the robot
            /*case 6:
                status = arm.extendArm(14, 0.2);
                break;*/

            // Drive backwards 1 foot
            case 6:
                status = drive.driveDistanceWithAngle(0, 1, 0.5);            
                break;

            // Rotate the robot 43 degrees
            case 7:
                status = drive.rotateRobot(Math.toRadians(43));            
                break;

            // Rotate the wheels back to zero before driving forward
            case 8:
                status = drive.rotateWheelsToAngle(0);            
                grabber.intakeOutake(true, false);
                break;

            // Drive back 4 feet and pick up a note
            case 9:
                if (intakeStatus == Robot.CONT) {
                    intakeStatus = grabber.intakeOutake(true, false);
                }
                
                if (driveStatus == Robot.CONT) {
                    driveStatus = drive.driveDistanceWithAngle(0, 4, 0.5);
                }
                
                if (intakeStatus == Robot.DONE && driveStatus == Robot.DONE) {
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
            
                break;

            // Finished routine, reset variables, stop motors, and return done
            default:
                shooter.stopShooting();
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
     * <p> Moves the arm so the shooter is aiming at the speaker (located in the left position)
     * <p> -26 degrees from horizontal
     * <p> Fully retracts arm
     * <p> First rotates, then retracts the arm
     * <p> This currently takes 11 seconds for a full auto cycle
     * @return Robot status, CONT or DONE
     */
    public int speakerPositionLeft() {
        if(firstTime == true) {
            firstTime = false;
            intakeStatus = Robot.CONT;
            driveStatus = Robot.CONT;
            step = 1;
        }

        switch(step) {            
            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            /*case 1:
                shooter.spinup();
                status = arm.rotateArm(333);
                break;
                        
            // Shoot the note by running the grabber
            case 2:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(333);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 1 second(s)
            case 3:
                status = autoDelay(1);
                arm.maintainPosition(333);
                break;

            // Rotate the arm to it's resting position and turn off the shooter and Switch the grabber to intake mode
            case 4:
                status = arm.rotateArm(322);
                break;

            // Extend the arm so the wood holding block falls into the robot, and so the arm is in the shooting position
            case 5:
                status = arm.extendArm(8, 0.25);
                break;*/

            // Drive backwards 1 foot
            case 999999999:
                /*if (intakeStatus == Robot.CONT) {
                    intakeStatus = grabber.intakeOutake(true, false);
                }*/

                intakeStatus = Robot.DONE;
                
                if (driveStatus == Robot.CONT) {
                    driveStatus = drive.driveDistanceWithAngle(0, 1, 0.3);
                }
                
                if (intakeStatus == Robot.DONE && driveStatus == Robot.DONE) {
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
            
                break;

            // Rotate to zero degrees
            case 1:
                //status = drive.ro(Math.toRadians(-45));
                //driveStatus = drive.driveDistanceWithAngle(-90, 2, 0.2);
                Measure<Angle> angleMeasurement = Units.Radians.of(0);  // Get the desired angle as a Measure<Angle> 
                Translation2d vect = new Translation2d(0.0, new Rotation2d(angleMeasurement));  // Create Translation2d for rotateWheels
                status = drive.rotateWheelsNoOpt(vect.getX(), vect.getY(), 0.0);  // Rotate wheels to 0 radians
                break;

            // Drive back to the speaker
            /*case 7:
                status = drive.driveDistanceWithAngle(0, -4, 0.3);
                break;

            // Rotate the arm so it's in the shooting position
            case 8:
                status = arm.rotateArm(335); // Use 343 if not driving to speaker          
                break;

            // Shoot the note
            case 9:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(335);
                status = autoDelay(1);
                break;*/
            
            // Finished routine, reset variables, stop motors, and return done
            default:
                shooter.stopShooting();
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

            // Wait for the thooter motors to spin up
            case 2:                
                arm.maintainPosition(SHOOT1_ANGLE);
                //status = autoDelay(1);
                status = Robot.DONE;
                break;
            
            // Start the grabber and keep the arm in shooting position
            default:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
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
