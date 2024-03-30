// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // STATUS CODES
	public static final int FAIL = -1;
	public static final int PASS =  1;
	public static final int DONE =  2;
	public static final int CONT =  3;

	// Object creation
  NetworkTableInstance FCSInfo;
	PoseEstimation       position;
	CustomTables         nTables;
  AprilTags            apriltags;
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
  private boolean autoShooterState = false;

	// Auto path
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private String m_autoSelected;
  
  // Statuses for each "module" 
  private int shooterStatus     = CONT;
  private int grabberStatus     = CONT;
  private int moveStatus        = CONT;
  private int armStatus         = CONT;
  private int restingStatus     = CONT;
  
  // Statuses for AprilTag targeting
  private int apriltagArmStatus     = CONT;
  private int apriltagAlignedStatus = CONT;
  
  private boolean shooterSpinning;
  private boolean limelightLED;

  private double startTime = 0;
  private int iterCount = 0;

  /* arm states */
  enum ArmState {TELEOP, CLIMB, AMP, REST, INTAKE, SHOOT};
  private ArmState armState = ArmState.TELEOP;

  /* TeleOp States */
  enum TeleopState { TELEOP, TARGET };
  private TeleopState teleopState = TeleopState.TELEOP;

	/**
	 * Constructor
	 */
	public Robot() {
    //System.out.println("[INFO] >> Initializing chooser(s)...");
    
    m_chooser = new SendableChooser<>();

		// Instance getters
    //System.out.println("[INFO] >> Initializing instance getters...");
        
		nTables  = CustomTables.getInstance();
    FCSInfo = NetworkTableInstance.getDefault();

		// Instance creation
    grabber   = Grabber.getInstance();
    apriltags = new AprilTags(nTables.getIsRedAlliance());
		drive     = new Drive(apriltags);
		controls  = new Controls();
		position  = new PoseEstimation(drive);
    shooter   = new Shooter();
    climber   = new Climber();
    arm       = new Arm();
		auto      = new Auto(drive, position, arm, grabber, shooter, apriltags);
    led       = new LED();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //System.out.println("[INFO] >> Robot init running.");

    // Start the camera server
    //System.out.println("[INFO] >> Starting camera server...");
    //CameraServer.startAutomaticCapture();
    
    // Auto selection
    //System.out.println("[INFO] >> Configuring auto...");
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
    limelightLED = false;
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
    System.out.println("[INFO] >> Auto mode selected.");
    m_autoSelected = m_chooser.getSelected();

    System.out.println("[INFO] >> Getting alliance color...");
        
    auto.isRed = this.nTables.getIsRedAlliance();
    
    System.out.println("[INFO] >> Getting alliance color...");
        
    if(!auto.isRed) {
        auto.allianceAngleModifier = -1;
    }
    
    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    status = Robot.CONT;

    System.out.println("[INFO] >> Auto program selected: " + m_autoSelected);    
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
    //System.out.println("[INFO] >> TeleOp mode selected.");

    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    status = Robot.CONT;

    shooterSpinning = false;
    //apriltags.setSpeakerPipeline();
    //System.out.println(apriltags.getDistanceToSpeakerFeet());

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
    System.out.println("[INFO] >> Test mode selected.");

    // Initialize Shuffleboard
    SmartDashboard.putNumber("Rotation Power", 0.0);

    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    status = Robot.CONT;

    //driveDistance = false;
    startTime = System.currentTimeMillis();
    iterCount = 0;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Read values from shuffleboard
    //double shooterPower = SmartDashboard.getNumber("Shooter Power", 0.0);
    //double grabberPower = SmartDashboard.getNumber("Grabber Power", 0.0);
    //double rotateAngle = SmartDashboard.getNumber("Rotation Angle", 0.0);
    //double power = SmartDashboard.getNumber("Rotation Power", 0.0);

    //apriltags.setSpeakerPipeline();
    //System.out.println(apriltags.calculateArmAngleToShoot());

    //drive.setAllRotateMotorPower(power);    

    //apriltags.setSpeakerPipeline();
    //if(drive.alignWithAprilTag() == DONE) {
    //  System.out.println("Finished in: " + (System.currentTimeMillis() - startTime) + "ms | " + iterCount + " iterations");
    //  startTime = System.currentTimeMillis();
    //  iterCount = 0;
    //}
    //else {
    //  iterCount++;
    //}

    /*if (armStatus == Robot.CONT) {
      armStatus = arm.extendArmToPosition(arm.ARM_INTAKE_POSITION, 0.3); //145
    }

    System.out.println(arm.getExtendPosition());*/

    //System.out.println(drive.getFLRot());

    //System.out.println(Math.toDegrees(drive.getBRAngle()));

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
    //SmartDashboard.putBoolean("Note Detected", grabber.noteDetected());

    // Test AprilTags
    //drive.alignWithAprilTag();
    System.out.println(apriltags.getDistanceToSpeakerFeet());

    // Test the auto selection
    //System.out.println("Selected auto: " + m_chooser.getSelected());

    // Test LEDs
    //ledControl();

    // Move the arm to a certain degree
    /*if (armStatus == Robot.CONT) {
      armStatus = arm.rotateArm(355);
    }
    else {
      armStatus = arm.maintainPosition(355);
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
    //System.out.println("Arm position: " + arm.getElevationPosition());
    //System.out.println("Arm extension position: " + arm.getExtendPosition());

    // Get drive controller values
    //System.out.println("Forward speed: " + controls.getForwardSpeed() + " Strafe speed: " + controls.getStrafeSpeed() + " Rotate speed: " + controls.getRotateSpeed());
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
    boolean targetSpeaker     = controls.toggleSpeakerTargeting();

    // Zeros the gyro
		if (zeroYaw == true) {
			drive.resetYaw();
		}

    // Targets the speaker
    if(targetSpeaker) {
      teleopState = TeleopState.TARGET;
      apriltags.setSpeakerPipeline();   // Set the pipeline depending on alliance color(0,ID4,Red, 1,ID7,Blue)
    }
    
    if(teleopState == TeleopState.TELEOP) {
      drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, fieldDrive);

      teleopState = TeleopState.TELEOP;
    }
    else if(teleopState == TeleopState.TARGET) {
      int targetStatus = auto.targetSpeaker();
      
      if(targetStatus == DONE) {      // Done rotating, drive forward
        teleopState = TeleopState.TELEOP;
      }
      else if(targetStatus == CONT) { // Not done rotating
        teleopState = TeleopState.TARGET;
      }
      else if(targetStatus == FAIL) { // Can't find AprilTag
        teleopState = TeleopState.TELEOP;
      }
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
	}

  /**
	 * Controls the LEDs
	 */
  private void ledControl() {
    boolean hasNote = grabber.noteDetected();
    boolean runningIntake = controls.runIntake();
    boolean partyMode = controls.enablePartyMode();
    
    if(controls.toggleLimelightLED()) {
      limelightLED = !limelightLED;
    }
      apriltags.setLED(limelightLED);

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
          else {
            arm.testElevate(0);
          }

          //Move the arm out/in incrementally
          //System.out.println("Arm extension position: " + arm.getExtendPosition());
          if(controls.extendArm()) {
            arm.testExtend(0.35);
          } 
          else if(controls.retractArm()) {
            arm.testExtend(-0.35);
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
          else if (controls.moveToIntakePosition())  {
              armState = ArmState.INTAKE;
          }
          else if (controls.enableShooter())  {
              armState = ArmState.SHOOT;
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
          armStatus = arm.extendToRest();

          if (armStatus == Robot.DONE) {
              armState = ArmState.TELEOP;
          }
      }
      else if (armState == ArmState.INTAKE)
      {
          armStatus = arm.extendToIntake();

          if (armStatus == Robot.DONE) {
              armState = ArmState.TELEOP;
          }
      }
      else if (armState == ArmState.SHOOT)
      {
          // Hand over control of the arm to shooterControl
          if (controls.enableShooter() == false) {
              armState = ArmState.TELEOP;
          }
          else {
            armState = ArmState.SHOOT;
          }
      }
  }

  /**
	 * Controls the shooter in TeleOp
	 */
	private void shooterControl() {
    boolean enableShooter = controls.enableShooter();
    boolean enableAutoShooter = controls.enableAutoShoot();

    // Shoot a note
    if (enableShooter == true) {
      shooterState = true;
      auto.teleopShoot(enableShooter);
    }
    
    if(enableAutoShooter == true && enableShooter == false) {
      autoShooterState = true;
      auto.apriltagShoot(enableAutoShooter);
    }
    
    // If the drivers have just stopped shooting
    if (shooterState == true && enableShooter == false) {
      shooterState = false;
      auto.resetTeleopShoot();
      shooter.stopShooting();
    }
    else if (autoShooterState == true && enableAutoShooter == false) {
      autoShooterState = false;
      auto.resetAutoShoot();
      shooter.stopShooting();
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
      grabber.intakeOutake(controls.runIntake(), controls.ejectNote(), false);
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
      climber.runLeftClimber(climber.PRECISION_CLIMB_POWER); //CLIMB_POWER
      climber.setRightClimberPower(0);
    }
    else if (runLeft == false && runRight){
      climber.setLeftClimberPower(0);
      climber.runRightClimber(climber.PRECISION_CLIMB_POWER); //CLIMB_POWER
    }
    else {
      climber.runLeftClimber(0);
      climber.runRightClimber(0);
    }
  }

  /*private void targetSpeakerTag() {
    double armAngle;  // Angle to rotate to

    // Check if the robot is out of range
    if(apriltags.outOfRange()) {
      led.apriltagOutOfRangeColor();
      led.updateLED();
      
      return; // Don't do anything, out of range
    }

    // Final checks if there's no driver input(robot stop)
    if(
      controls.getForwardSpeed() == 0 && 
      controls.getRotateSpeed() == 0 && 
      controls.getStrafeSpeed() == 0
    ) {
      armAngle = apriltags.calculateArmAngleToShoot(); // Get arm angle

      // Goto and maintain arm shooting angle
      if(apriltagArmStatus == CONT) {
        apriltagArmStatus = arm.rotateArm(armAngle);
      } 
      else {
        arm.maintainPosition(armAngle);
      }

      // Face AprilTag(always rotate in case of bumping?)
      if(apriltagAlignedStatus == CONT) {
        apriltagAlignedStatus = drive.alignWithAprilTag();  // Rotate to face the tag
      }

      if(apriltagAlignedStatus == DONE && apriltagArmStatus == DONE){
        led.apriltagReadyToShootColor();
        led.updateLED();
      }
    }
    // Still moving, just rotate arm and set LEDs to orange(in range but moving)
    else {
      armAngle = apriltags.calculateArmAngleToShoot(); // Get arm angle
      apriltagArmStatus = arm.rotateArm(armAngle);     // Rotate arm
      
      // Set LEDs to orange
      led.apriltagInRangeMovingColor();
      led.updateLED();
    }
  }
*/
  private void testTeleopDrive() {
    double rotateSpeed = controls.getRotateSpeed();
    double strafeSpeed = controls.getStrafeSpeed();
    double forwardSpeed = controls.getForwardSpeed();

    drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, false);

  }
}