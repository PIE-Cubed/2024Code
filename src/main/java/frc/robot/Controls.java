// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



/****************************************************************************************** 
*
*    IMPORTANT NOTE!!!
* 
******************************************************************************************/
/***
 * The Xbox controller DPAD is known as "POV", and the input is retrieved from the getPOV function.
 * The getPOV function returns an integer from -1 to 7, going clockwise.
 * The button assignments are as follows:
 * -1 = nothing pressed
 * 0 = top
 * 1 = top right
 * 2 = right
 * 3 = bottom right
 * 4 = bottom
 * 5 = bottom left
 * 6 = left
 * 7 = top left
 */

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Start of the Controls class
 */
public class Controls {
	// CONSTANTS
	private final int DRIVE_ID = 0;
	private final int MANIPULATOR_ID = 1;

	// Controller object declaration
	private XboxController driveController;
	private XboxController manipulatorController;

	// Rate limiters
	//private SlewRateLimiter xLimiter;
	//private SlewRateLimiter yLimiter;
	//private SlewRateLimiter rotateLimiter;

	/**
	 * The constructor for the Controls class
	 */
	public Controls() {
		// Instance Creation
		driveController = new XboxController(DRIVE_ID);
		manipulatorController = new XboxController(MANIPULATOR_ID);

		// Create the rate limiters
		//xLimiter      = new SlewRateLimiter(12); // -6 to 6 in two seconds
		//yLimiter      = new SlewRateLimiter(12); // -6 to 6 in two seconds
		//rotateLimiter = new SlewRateLimiter(6 * Math.PI);
	}



	/****************************************************************************************** 
    *
    *    DRIVE FUNCTIONS
    * 
    ******************************************************************************************/
	/**
	 * Gets the forward speed
	 * <p>Forward is positive to match chassis speed standards
	 * <p>This measures rotatation around the Y axis, which is effectively translation on the X axis
	 * 
	 * @return forwardSpeed
	 */
	public double getForwardSpeed() {
		double speed;
		double power = -1 * driveController.getLeftY();
		power = Math.pow(power, 7);

		// Turns the power into a speed
		if(enablePrecisionDrive()){
			speed = power * Drive.POWER_SPEED_RATIO_MPS_PREICISION;	// Use precision mode curve
		} else {
			speed = power * Drive.POWER_SPEED_RATIO_MPS;	// Use regular curve
		}

		// Limits the acceleration when under driver control
		//speed = xLimiter.calculate(speed);

		return speed;
	}

	/**
	 * Gets the strafe speed
	 * <p>Left is negative, converted to chassis speed standards in Drive.teleopDrive()
	 * <p>This measures rotatation around the X axis, which is effectively translation on the Y axis
	 * 
	 * @return strafeSpeed
	 */
	public double getStrafeSpeed() {
		double speed;
		double power = driveController.getLeftX();
		power = Math.pow(power, 7);

		// Turns the power into a speed
		if(enablePrecisionDrive()){
			speed = power * Drive.POWER_SPEED_RATIO_MPS_PREICISION;	// Use precision mode curve
		} else {
			speed = power * Drive.POWER_SPEED_RATIO_MPS;	// Use regular curve
		}

		// Limits the acceleration when under driver control
		//speed = yLimiter.calculate(speed);

		return speed;
	}

	/**
	 * Gets the rotate speed
	 * <p>Clockwise is positive, converted to chassis speed standards in Drive.teleopDrive()
	 * <p>This measures rotatation around the Z axis
	 * 
	 * @return rotateSpeed
	 */
	public double getRotateSpeed() {
		double speed;
		double power = driveController.getRightX();
		power = Math.pow(power, 7);

		// Turns the power into a speed
		speed = power * Drive.MAX_ROTATE_SPEED;	// Use regular curve in radians per second
		
		// Limits the acceleration when under driver control
		//speed = rotateLimiter.calculate(speed);

		return speed;
	}
	
	/**
	 * Pressing left joystick will zero yaw in case of emergency
	 * 
	 * @return zeroYaw
	 */
	public boolean zeroYaw() {
		return driveController.getLeftStickButtonPressed();
	}
	
	/**
	 * Holding right trigger will enable precision control
	 * 
	 * @return precisionControl
	 */
	public boolean enablePrecisionDrive() {
		return driveController.getRightTriggerAxis() > 0.05;
	}

	// Targeting

	/***
	 * Pressing the A button will toggle speaker targeting mode
	 * @return Drive controller A button pressed
	 */
	public boolean toggleSpeakerTargeting() {
		return driveController.getAButtonPressed();
	}

	// Stage Functions

	/***
	 * Pressing the X button will toggle trap sequence
	 * @return Drive controller X button pressed
	 */
	public boolean toggleTrapSequence() {
		return driveController.getXButtonPressed();
	}

	/***
	 * Pressing the Y button will start auto climb
	 * @return Drive controller Y button pressed
	 */
	public boolean autoClimb() {
		return driveController.getYButtonPressed();
	}

	//D-Pad

	/***
	 * Pressing the DPAD UP button will reset the gyro
	 * @return Drive controller DPAD UP button pressed
	 */
	public boolean resetGyro() {
		return driveController.getPOV() == 0;
	}

	/***
	 * Pressing the DPAD DOWN button will lock the drive wheels
	 * @return Drive controller DPAD DOWN button pressed
	 */
	public boolean lockDriveWheels() {
		return driveController.getPOV() == 4;
	}

	// Lights

	/***
	 * Pressing the Left Bumper button will amplify signal lights
	 * @return Drive controller Left Bumper button pressed
	 */
	public boolean amplifySignalLights() {
		return driveController.getLeftBumperPressed();
	}

	/***
	 * Pressing the Right Bumper button will toggle source lights
	 * @return Drive controller Right Bumper button pressed
	 */
	public boolean toggleSourceLights() {
		return driveController.getRightBumperPressed();
	}


	/****************************************************************************************** 
    *
    *    MANIPULATOR FUNCTIONS
    * 
    ******************************************************************************************/
	// Intake & outtake
	/***
	 * Holding the A button will make the intake move to the ground position
	 * @return Manipulator controller A button held down
	 */
	public boolean enableGroundIntake() {
		return manipulatorController.getAButton();
	}

	/***
	 * Holding the left trigger will run the intake motors
	 * @return Manipulator controller left trigger held in
	 */
	public boolean runIntake() {
		return manipulatorController.getLeftTriggerAxis() > 0.05;
	}

	/***
	 * Pressing the left bumper will eject the note from the intake
	 * @return Manipulator controller left bumper pressed
	 */
	public boolean ejectNote() {
		return manipulatorController.getLeftBumperPressed();
	}

	/***
	 * Pressing the right bumper will shoot the note softly
	 * @return Manipulator controller right bumper pressed
	 */
	public boolean dumpShot() {
		return manipulatorController.getRightBumper();
	}
	
	// Shooting
	/***
	 * Holding the right trigger will run the shooter motors
	 * @return Manipulator controller right trigger held in
	 */
	public boolean enableShooter() {
		return manipulatorController.getRightTriggerAxis() > 0.05;
	}

	/***
	 * Pressing the Start button will start the shooter wheels
	 * @return Manipulator controller Start button pressed
	 */
	public boolean startShooterWheels() {
		return manipulatorController.getStartButtonPressed();
	}

	/***
	 * Pressing the Back button will stop the shooter wheels
	 * @return Manipulator controller Stop button pressed
	 */
	public boolean stopShooterWheels() {
		return manipulatorController.getBackButtonPressed();
	}

	// Positioning
	/***
	 * Pressing the B button will move the arm to the AMP position
	 * @return Manipulator controller B button pressed
	 */
	public boolean moveToAmpPosition() {
		return manipulatorController.getBButtonPressed();
	}

	/***
	 * Pressing the X button will move the arm to the resting position
	 * @return Manipulator controller X button pressed
	 */
	public boolean moveToRestPosition() {
		return manipulatorController.getXButtonPressed();
	}

	// Arm movement
	/***
	 * Pressing the DPAD UP button will move the arm upwards
	 * @return Manipulator controller DPAD UP button pressed
	 */
	public boolean moveArmUp() {
		return manipulatorController.getPOV() == 0;
	}

	/***
	 * Pressing the DPAD DOWN button will move the arm downward
	 * @return Manipulator controller DPAD DOWN button pressed
	 */
	public boolean moveArmDown() {
		return manipulatorController.getPOV() == 4;
	}

	/***
	 * Pressing the DPAD RIGHT button will retract the arm
	 * @return Manipulator controller DPAD RIGHT button pressed
	 */
	public boolean retractArm() {
		return manipulatorController.getPOV() == 2;
	}

	/***
	 * Pressing the DPAD LEFT button will extend the arm
	 * @return Manipulator controller DPAD LEFT button pressed
	 */
	 boolean extendArm() {
		return manipulatorController.getPOV() == 6;
	}



	/****************************************************************************************** 
    *
    *    MISC FUNCTIONS
    * 
    ******************************************************************************************/
	/**
	 * Checks if the start button is pressed on the drive controller.
	 * 
	 * @return startButtonPressed
	 */
	public boolean autoKill() {
		return driveController.getStartButtonPressed();
	}

}
// End of the Controls class
