// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  Shooter        shooter;
  Arm            arm;
  Grabber        grabber;

	// Constants
	private final double ROTATE_SPEED_OFFSET = -0.16;

	// Variables
	private int status = CONT;
	private boolean firstTime = true;
  private boolean shooterState = false;

	// Auto path
	private static final String autoMode = "Wall";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	// Auto Delay
	private long delaySec = 0;

  // Distance test status
  private boolean driveDistance;  /// Whether the robot is at the drive step of the test
  
  // Statuses for each "module" 
  private int shooterStatus = CONT;
  private int grabberStatus = CONT;
  private int moveStatus = CONT;
  private int armStatus = CONT;

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
    shooter  = new Shooter();
    arm      = new Arm();
    grabber  = Grabber.getInstance();
		auto     = new Auto(drive, position, arm, grabber, shooter);

		// Instance getters
		nTables  = CustomTables.getInstance();

    LimelightHelpers.setLEDMode_ForceOff("limelight");
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Speaker Center", "Speaker Center");
    m_chooser.addOption("Speaker Left", "Speaker Left");
    m_chooser.addOption("Speaker Right", "Speaker Right");
    SmartDashboard.putData("Auto choices", m_chooser);

    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    status = Robot.CONT;
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
    //position.updatePoseTrackers();
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
    auto.selectedAuto = m_autoSelected;
    
    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    status = Robot.CONT;

    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);

    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (status == Robot.CONT) {
      status = auto.speakerPosition();
    }

    if (moveStatus == Robot.CONT) {
      switch (m_autoSelected) {
        case "Speaker Center":
          break;

        case "Speaker Left":
          //moveStatus = drive.driveDistanceWithAngle(-35, 3, 0.1);
          break;
        
        case "Speaker Right":
          break;

        default:
          // Put default auto code here
          break;
      }
    }

    //SmartDashboard.putNumber("Extend position", arm.getExtendPosition());
    //SmartDashboard.putNumber("Arm angle", arm.getElevationPosition());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    status = Robot.CONT;

    // Turn on the shooter motors
    //shooter.spinup();    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Allows for driving the robot
    wheelControl();

    // Allows for controlling the arm
    armControl();

    // Allows for shooting notes
    shooterControl();
    
    // Allows for controlling the grabber
    grabberControl();

    //grabber.intakeOutake(true, false);
    //armStatus = arm.rotateArm(342);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Initialize Shuffleboard
    //SmartDashboard.putNumber("Shooter Power", 0.0);
    //SmartDashboard.putNumber("Grabber Power", 0.0);

    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    status = Robot.CONT;

    //driveDistance = false;
  }

  private static int increment = 0;

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Read values from shuffleboard
    //double shooterPower = SmartDashboard.getNumber("Shooter Power", 0.0);
    //double grabberPower = SmartDashboard.getNumber("Grabber Power", 0.0);

    //testTeleopDrive();
  
    // Test shooter
    //shooter.startShooting(shooterPower);

    // Test grabber
    // Does not automatically stop the grabber or check for a note
    //grabber.setMotorPower(grabberPower);

    // Automatically stops the grabber when a note is detected
    /*
    if (grabberStatus == Robot.CONT) {
      grabberStatus = grabber.intakeOutake(true, false);
    }*/

    // Retrieve RGB, IR, and proximity values from the color sensor
    //grabber.testColorSensor();

    // Move the arm to a certain degree
    /*if (armStatus == Robot.CONT) {
      armStatus = arm.rotateArm(54);
    }
    else {
      armStatus = arm.maintainPosition(54);
    }*/

    // Test driving at an angle
    if(status == Robot.CONT) {
      status = drive.testAngleDrive(0, 3, 0.3);
    }

    // 60 from horizontal, arm extends 4in
    /*
    System.out.println("Extension Encoder: " + Math.toDegrees(arm.getExtendPosition()));

    arm.testExtend(0.05);    
    System.out.println("Elevation Encoder" + Math.toDegrees(arm.getElevationPosition()));

    if (increment == 0)
      System.out.println("ArmStatus: " + armStatus);
    if(armStatus != DONE) {
      armStatus = arm.rotateArm(Math.toRadians(45));  
    }
    
    increment++;
    */
    
    // Test arm elevation
    //arm.testElevate();
    //System.out.println("Elevation Encoder", Math.toDegrees(arm.getElevationPosition()));

    // Test driving at an angle
    /*
    if (status == Robot.CONT) {
      status = drive.driveDistanceWithAngle(0, -2, 0.3);
    }*/

    // Test power and velocity
    /*
    drive.setAllDriveMotorPower(MathUtil.clamp(power, -1, 1));
    drive.printPowerandVelocity();*/

    // Rotate drive wheels to zero
    /*if (status == Robot.CONT) {
      Measure<Angle> angleMeasurement = Units.Radians.of(0);  // Get the desired angle as a Measure<Angle> 
      Translation2d vect = new Translation2d(0.0, new Rotation2d(angleMeasurement));  // Create Translation2d for rotateWheels
      status = drive.rotateWheelsNoOpt(vect.getX(), vect.getY(), 0.0);  // Rotate wheels to 0 radians
    }*/    

    // Test distance
    /*double distance = 5;

    // Are all wheels at 0 radians and driveDistance is true
    if(driveDistance == false)  {
        // Rotate all wheels to 0 radians
        if (status == Robot.CONT)  {
          /*  TJM
           *   I don't think this is what you want.  I think you are trying to rotate the wheels only
           *   and not provide power to drive wheels.  rotateWheels() calls SetDesiredState() which will 
           *   rotate the wheels and provide drive power.  If you want to go to 0 degrees for instance you
           *   could set x=0 and z=0 and make y(forward power) small like .01.  
           *   if x = 0 and y = .01 the angle would be 0 and wheel power would be small.
           *   Perhaps a better way is  to 
           *        use a rotate program like we did earlier OR
           *        make a rotate program similar to teleop drive that only gives power to rotate motor
           *            You could pass in a Rotation2d(angleRadians) and then directly set the SwerveModuleStates()
           *            with forward power = 0 and all wheels getting the angle passed in
           *   We can talk over on Tuesday or before if this doesn't make sense.
           * 
           */
          /*Measure<Angle> angleMeasurement = Units.Radians.of(0);  // Get the desired angle as a Measure<Angle> 
          Translation2d vect = new Translation2d(0.0, new Rotation2d(angleMeasurement));  // Create Translation2d for rotateWheels
          status = drive.rotateWheelsNoOpt(vect.getX(), vect.getY(), 0.0);  // Rotate wheels to 0 radians
      }
      else if (status == Robot.DONE)  {
            status = Robot.CONT;  // Reset status
            driveDistance = true; // Start the distance test
        }
    } 
    else {
        // At 0 radians, start/continue the test
        if (status == Robot.CONT)  {
            //status = drive.driveDistance((Math.PI * 2.875) / 12, 0.05);  // ~0.75ft for wheel rotation tests
            status = drive.driveDistance(distance, 0.075);
        }
    }*/

    // Test AprilTags
    //drive.testAprilTagID();
    //drive.testAprilTagXY();
    //drive.testAprilTagPipeline(0);
    //drive.testMegaTagPose(0);
    //System.out.println(drive.getDistanceToAprilTagMeters(0, 7));
    //drive.alignWithAprilTag(1, 4);  // Pipeline 1 has a mask for ID 4

    // Test rotation
    //double deg = SmartDashboard.getNumber("Angle(Deg)", 0);
    //int status = drive.rotateRobot(Math.toRadians(deg));
    //if(status == DONE) System.out.println("Done");

    // Test controller
    //SmartDashboard.putNumber("Controller L", controls.getForwardSpeed());
  }

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
		// Gets the Drive Values
		double  rotateSpeed       = controls.getRotateSpeed();
		double  strafeSpeed       = controls.getStrafeSpeed();
		double  forwardSpeed      = controls.getForwardSpeed();

		// Gets Manipulator values
		boolean zeroYaw           = controls.zeroYaw();
    boolean fieldDrive        = controls.toggleFieldDrive();

    // Zeros the gyro
		if (zeroYaw == true) {
			drive.resetYaw();
		}
    
		// Calculated line of best fit for relationship between rotate speed and drift angle
    // Think its used to travel straight when rotating
    // Will allegedly revisit later to adjust to new motors 
		/**
     *  double angleTransform = ROTATE_SPEED_OFFSET * rotateSpeed;
     * Translation2d velocity = new Translation2d(forwardSpeed, strafeSpeed);
     * Translation2d newVelocity = velocity.rotateBy(new Rotation2d(angleTransform));
     * double newXVel = newVelocity.getX();
     * double newYVel = newVelocity.getY();
     * 
     * drive.teleopDrive(newXVel, newYVel, rotateSpeed, true);
     */

		drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, fieldDrive);
	}

  /**
	 * Controls the arm in TeleOp
	 */
	private void armControl() {
    // Move the arm up/down incrementally
    if(controls.moveArmUp()) {
      //arm.rotateArmIncrement(true, false);
      arm.testElevate(-0.5);
    }
    else if (controls.moveArmDown()) {
      //arm.rotateArmIncrement(false, true);
      arm.testElevate(0.5);
    }
    else{
      arm.testElevate(0);
    }

    // Extend / retract the arm
    if(controls.extendArm()){
      arm.testExtend(0.2);
    }
    else if(controls.retractArm()) {
      arm.testExtend(-0.2);
    }
    else {
      arm.testExtend(0);
    }


	}

  /**
	 * Controls the shooter in TeleOp
	 */
	private void shooterControl() {
    // Shoot a note
    if (controls.enableShooter() == true) {
      shooterState = true;
      auto.teleopShoot();
    }
    
    // If the divers have just stopped shooting, disable the shooter motors
    if (shooterState == true && controls.enableShooter() == false) {
      shooterState = false;
      shooter.stopShooting();
      auto.resetTeleopShoot();
    }
	}

  /**
	 * Controls the grabber in TeleOp
	 */
	private void grabberControl() {
    // Start the grabber in ground mode 
    if(shooterState == false) {
      grabber.intakeOutake(controls.runIntake(), controls.ejectNote());
    } 

    /*else if (controls.runIntake()){
      grabber.setMotorPower(grabber.INTAKE_POWER);
    }
    else {
      grabber.setMotorPower(0);
    }*/
	}

  /**
   * 
   */
 // private void grabberControl() {
 //   boolean intake = controls.runIntake();
 //   boolean outtake = controls.ejectNote();
 //   
 //   grabber.intakeOutake(intake, outtake);
 // }
//
 // /**
 // private void armControl() {
 //   boolean rotateUp     = controls.moveArmUp();
 //   boolean rotateDown   = controls.moveArmDown();
//
 //   boolean extendArm    = controls.extendArm();
 //   boolean retractArm   = controls.retractArm();
//
 //   arm.rotateArmIncrement(rotateUp, rotateDown);
 //   arm.moveArmIncrement(extendArm, retractArm);
 // }

  /**
   * 
   */
  public void testTeleopDrive() {
    double rotateSpeed = controls.getRotateSpeed();
    double strafeSpeed = controls.getStrafeSpeed();
    double forwardSpeed = controls.getForwardSpeed();

    drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, false);

  }

  /**
   * 
   */
  private void testAprilTag() {
    // Using NetworkTables
    NetworkTable aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight");
    if(aprilTagTable.getEntry("tv").getBoolean(false)){
      System.out.println("Has Target: " + aprilTagTable.getEntry("tv").getBoolean(false));
      double[] targetPoseRobotSpace = aprilTagTable.getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
    
      /* Print the AprilTag's pose in robotspace
       * Prints out as: x, y, z, rx, ry, rz
       */
      for(double num : targetPoseRobotSpace) {
        System.out.print(num + ", ");
      }
    }
  }
}
