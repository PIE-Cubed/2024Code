// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  public LimelightTarget_Fiducial Fiducial;

  // ERROR CODES
	public static final int FAIL = -1;
	public static final int PASS =  1;
	public static final int DONE =  2;
	public static final int CONT =  3;

	// Object creation
	PoseEstimation position;
	CustomTables   nTables;
	Controls       controls;
	Drive          drive;
	Auto           auto;

	// Constants
	private final double ROTATE_SPEED_OFFSET = -0.16;

	// Variables
	private int status = CONT;
	private boolean firstTime = true;

	// Auto path
	private static final String autoMode = "Wall";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	// Auto Delay
	private long delaySec = 0;

	/**
	 * Constructor
	 */
	public Robot() {
    m_chooser = new SendableChooser<>();
    Fiducial = new LimelightTarget_Fiducial();

		// Instance creation
		drive    = new Drive();
		controls = new Controls();
		position = new PoseEstimation(drive);
		auto     = new Auto(drive, position);

		// Instance getters
		nTables  = CustomTables.getInstance();
	}

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    position.updateOdometry();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    wheelControl();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /**
	 * Controls the wheels in TeleOp
	 */
	private void wheelControl() {
		// Variables
		// Gets the Drive Values
		double  rotateSpeed       = controls.getRotateSpeed();
		double  strafeSpeed       = controls.getStrafeSpeed();
		double  forwardSpeed      = controls.getForwardSpeed();
		boolean precisionDrive    = controls.enablePrecisionDrive();

		// Gets the Manipulator values
		boolean zeroYaw           = controls.zeroYaw();

		if (zeroYaw == true) {
			drive.resetYaw();
		}

		if (precisionDrive)  {
				drive.teleopDrive(forwardSpeed / 4, strafeSpeed / 4, rotateSpeed / 4, true);
			}
			else {
				// Calculated line of best fit for relationship between rotate speed and drift angle
				double angleTransform = ROTATE_SPEED_OFFSET * rotateSpeed;
				Translation2d velocity = new Translation2d(forwardSpeed, strafeSpeed);
				Translation2d newVelocity = velocity.rotateBy(new Rotation2d(angleTransform));

				double newXVel = newVelocity.getX();
				double newYVel = newVelocity.getY();

				drive.teleopDrive(newXVel, newYVel, rotateSpeed, true);
			}
	}
}
