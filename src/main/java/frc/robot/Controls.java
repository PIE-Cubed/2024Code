// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Start of the Controls class
 */
public class Controls {
	// CONSTANTS
	private final int DRIVE_ID = 0;

	// Controller object declaration
	private XboxController driveController;

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
		power = Math.pow(power, 3);

		// Turns the power into a speed
		speed = power * Drive.POWER_SPEED_RATIO_MPS;

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
		power = Math.pow(power, 3);

		// Turns the power into a speed
		speed = power * Drive.POWER_SPEED_RATIO_MPS;

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
		power = Math.pow(power, 3);

		// Turns the power into a speed
		speed = power * Drive.MAX_ROTATE_SPEED;

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

	/****************************************************************************************** 
    *
    *    MISC FUNCTIONS
    * 
    ******************************************************************************************/
	/**
	 * Checks if the start button is pressed.
	 * 
	 * @return startButtonPressed
	 */
	public boolean autoKill() {
		return driveController.getStartButtonPressed();
	}
}
// End of the Controls class
