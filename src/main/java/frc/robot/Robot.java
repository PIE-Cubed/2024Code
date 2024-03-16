// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Climber;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public LimelightTarget_Fiducial Fiducial;

  // STATUS CODES
	public static final int FAIL = -1;
	public static final int PASS =  1;
	public static final int DONE =  2;
	public static final int CONT =  3;

	// Object creation
  NetworkTableInstance FCSInfo;
	PoseEstimation       position;
	CustomTables         nTables;
	Controls             controls;
  Shooter              shooter;
  Grabber              grabber;
  Climber              climber;
	Drive                drive;
	Auto                 auto;
  Arm                  arm;
  LED                  led;

	// Variables
	private int status = CONT;
	private boolean firstTime = true;
  private boolean shooterState = false;

	// Auto path
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private String m_autoSelected;
  
  // Statuses for each "module" 
  private int shooterStatus = CONT;
  private int grabberStatus = CONT;
  private int moveStatus    = CONT;
  private int armStatus     = CONT;
  private int restingStatus = CONT;

  private boolean shooterSpinning;

  /* arm states */
  enum ArmState {TELEOP, CLIMB, AMP, REST};
  private ArmState armState = ArmState.TELEOP;

	/**
	 * Constructor
	 */
	public Robot() {
    m_chooser = new SendableChooser<>();
    Fiducial = new LimelightTarget_Fiducial();

		// Instance getters
		nTables  = CustomTables.getInstance();
    FCSInfo = NetworkTableInstance.getDefault();

		// Instance creation
    grabber  = Grabber.getInstance();
		drive    = new Drive();
		controls = new Controls();
		position = new PoseEstimation(drive);
    shooter  = new Shooter();
    climber  = new Climber();
    arm      = new Arm();
		auto     = new Auto(drive, position, arm, grabber, shooter);
    led      = new LED();

    // Turn off the limelight LEDs so they don't blind people
    LimelightHelpers.setLEDMode_ForceOff("limelight");
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Start the camera server
    CameraServer.startAutomaticCapture();
    
    // Auto selection
    m_chooser.setDefaultOption("Speaker Center Auto", "Speaker Center Auto");
    m_chooser.addOption("Amp Side Auto", "Amp Side Auto");
    m_chooser.addOption("Feed Side Auto", "Feed Side Auto");
    m_chooser.addOption("No Auto", "No Auto");
    SmartDashboard.putData("Auto Selector", m_chooser);

    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    status = Robot.CONT;

    shooterSpinning = false;
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
    //auto.selectedAuto = m_autoSelected;

    auto.isRed = this.nTables.getIsRedAlliance();
    
    if(!auto.isRed) {
        auto.allianceAngleModifier = -1;
    }
    
    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    status = Robot.CONT;

    System.out.println("Auto selected: " + m_autoSelected);    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (status == Robot.CONT) {
      switch (m_autoSelected) {
        case "Speaker Center Auto":
          status = auto.speakerPositionCenter();
          break;

        case "Amp Side Auto":
          status = auto.speakerPositionAmp();
          break;

        case "Feed Side Auto":
          status = auto.speakerPositionFeed();
          break;

        case "No Auto":
          status = Robot.DONE;
          break;
          
        default:
          System.out.println("[INFO] >> Running default auto.");
          //status = auto.speakerPositionCenter();
          break;
      }

      //status = auto.speakerPositionCenter();
    } 
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

    // Allows for controlling the grabber
    grabberControl();
    
    // Allows for shooting notes
    shooterControl();

    // Allows for controlling the climber
    climberControl();
    
    // Allows for controlling the LEDs
    ledControl();
    
    // Drivers check this to see if they grabbed a note
    SmartDashboard.putBoolean("Grabber has Note", grabber.noteDetected());
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
    SmartDashboard.putNumber("Rotation Angle", 0.0);

    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    status = Robot.CONT;

    //driveDistance = false;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Read values from shuffleboard
    //double shooterPower = SmartDashboard.getNumber("Shooter Power", 0.0);
    //double grabberPower = SmartDashboard.getNumber("Grabber Power", 0.0);
    //double rotateAngle = SmartDashboard.getNumber("Rotation Angle", 0.0);

    //testTeleopDrive();
  
    // Test shooter
    //shooter.startShooting(shooterPower);

    // Test grabber
    // Does not automatically stop the grabber or check for a note
    //grabber.setMotorPower(1);

    // Automatically stops the grabber when a note is detected
    /*if (grabberStatus == Robot.CONT) {
      grabberStatus = grabber.intakeOutake(true, false);
    }*/

    // Retrieve RGB, IR, and proximity values from the color sensor
    //grabber.testColorSensor();

    // Test the auto selection
    //System.out.println("Selected auto: " + m_chooser.getSelected());

    // Test LEDs
    //ledControl();

    // Move the arm to a certain degree
    /*if (armStatus == Robot.CONT) {
      armStatus = arm.rotateArm(rotateAngle);
    }
    else {
      armStatus = arm.maintainPosition(rotateAngle);
    }*/

    // Test driving at an angle
    /* 
    if(status == Robot.CONT) {
      status = drive.testAngleDrive(0, 3, 0.3);
    }
    */
    
    // Test arm elevation
    //arm.testElevate();
    //System.out.println("Elevation Encoder", Math.toDegrees(arm.getElevationPosition()));

    // Test driving at an angle
    /*
    if (status == Robot.CONT) {
      status = drive.driveDistanceWithAngle(0, -2, 0.3);
    }*/

    // Get the arm extension position
    System.out.println("Arm extension position: " + arm.getExtendPosition());
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
    boolean fieldDrive        = controls.enableFieldDrive();

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
	 * Controls the LEDs
	 */
  private void ledControl() {
    boolean hasNote = grabber.noteDetected();
    boolean runningIntake = controls.runIntake();
    boolean partyMode = controls.enablePartyMode();

    if(partyMode) {             // If the robot is done climbing, top priority
      led.partyColor();           // Sets the color to rainbow
    } else if(hasNote) {        // If the grabber has a note, second priority
      led.capturedNoteColor();    // Sets the color to green
    } else if(runningIntake) {  // If the grabber is running(no note), third priority
      led.gettingNoteColor();     // Sets the color to orange
    } else {                    // Default state
      led.robolionsColor();       // Sets the color to blue-gold
    }
    
    led.updateLED();  // Update LEDs

  }
  
  /**
	 * Controls the arm in TeleOp
	 */
  private void armControl()
  {
      if (armState == ArmState.TELEOP)
      {
          // Move the arm up/down incrementally
          if(controls.moveArmUp()) {
              arm.testElevate(-0.5);
          }
          else if (controls.moveArmDown()) {
            arm.testElevate(0.5);
          }
          else{
            arm.testElevate(0);
          }

          //Move the arm out/in incrementally
          //System.out.println("Arm extension position: " + arm.getExtendPosition());
          if(controls.extendArm()) {
            arm.testExtend(0.25);
          } 
          else if(controls.retractArm()) {
            arm.testExtend(-0.25);
          } 
          else {
            arm.testExtend(0.0);
          }

          // next state
          if (controls.autoClimb())  {
              armState = ArmState.CLIMB;
          }
          else if (controls.moveToAmpPosition())  {
              armState = ArmState.AMP;
          }
          else if (controls.moveToRestPosition())  {
              armState = ArmState.REST;
          }

      }
      else if (armState == ArmState.CLIMB)
      {
          armStatus = arm.rotateArm(90);

          // next state
          if (armStatus == Robot.DONE)  {
            armState = ArmState.TELEOP;
          }
          else  {
              armState = ArmState.CLIMB;
          }
      }
      else if (armState == ArmState.AMP)
      {
          armStatus = arm.rotateArm(arm.ARM_AMP_POSITION_DEGREES);
          
          if (controls.moveToAmpPosition() && armStatus == Robot.DONE) {
              arm.maintainPosition(arm.ARM_AMP_POSITION_DEGREES);
          } 
          else {
              armState = ArmState.TELEOP;
          }
      }
      else if (armState == ArmState.REST)
      {
          armStatus = arm.rotateArm(arm.ARM_REST_POSITION_DEGREES);

          if (armStatus == Robot.DONE) {
              armState = ArmState.TELEOP;
          }
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
      arm.testElevate(0); // Let arm fall to rest position(bumper)
      auto.resetTeleopShoot();
    }

    // Start or stop the shooter wheels, the start button flips the current state
    if(controls.startShooterWheels()) {
      shooterSpinning = !shooterSpinning;
      if(shooterSpinning) {
        shooter.spinup();
      }
      else {
        shooter.spindown();
      }
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
      grabber.setMotorPower(grabber.FEED_POWER);
    }
    else {
      grabber.setMotorPower(0);
    }*/
	}

  /**
   * Controls the climber in TeleOp
   */
  private void climberControl() {
    boolean runLeft = controls.runLeftClimber();
    boolean runRight = controls.runRightClimber();

    if (runLeft && runRight) {      
      climber.runLeftClimber(climber.PRECISION_CLIMB_POWER);
      climber.runRightClimber(climber.PRECISION_CLIMB_POWER);
    }
    else if(runLeft && runRight == false) {
      climber.runLeftClimber(climber.CLIMB_POWER);
      climber.setRightClimberPower(0);
    }
    else if (runLeft == false && runRight){
      climber.setLeftClimberPower(0);
      climber.runRightClimber(climber.CLIMB_POWER);
    }
    else {
      climber.runLeftClimber(0);
      climber.runRightClimber(0);
    }
  }

  public void testTeleopDrive() {
    double rotateSpeed = controls.getRotateSpeed();
    double strafeSpeed = controls.getStrafeSpeed();
    double forwardSpeed = controls.getForwardSpeed();

    drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, false);

  }
}