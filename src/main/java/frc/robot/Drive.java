package frc.robot;

import com.fasterxml.jackson.annotation.ObjectIdGenerator.IdKey;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Start of the Drive class
 */
public class Drive {
    // Constants
    private final Translation2d FRONT_LEFT_LOCATION;
    private final Translation2d FRONT_RIGHT_LOCATION;
    private final Translation2d BACK_LEFT_LOCATION;
    private final Translation2d BACK_RIGHT_LOCATION;

	// When testing, 0.05 is the best max power for precision mode, Ausitn Approved!!
    public  static final double POWER_SPEED_RATIO_MPS   		 = 5.45;    // m/s / power
	public  static final double POWER_SPEED_RATIO_MPS_PREICISION = 0.45;	// m/s / power for precision mode
    public  static final double MAX_ROTATE_SPEED        		 = 1.5 * (2 * Math.PI); // Radians per second  (1.5 rotations/sec)
    private static final double MAX_WHEEL_SPEED         		 = 1;

    private final double AUTO_DRIVE_TOLERANCE        = 0.05;
    private final double AUTO_DRIVE_ROTATE_TOLERANCE = 0.05;    // Radians, ~3 degrees

    // Instance Variables
    private int     printCount             = 0;
    private int     autoPointIndex         = 0;
    private boolean autoPointAngled        = false; // Tracks if wheels have been angled before driving
    private boolean autoPointFirstTime     = true;
    private boolean driveDistanceFirstTime = true;
    private double  initXVelocity          = 0;
    private double  initYVelocity          = 0;
    private double  targetPosition         = 0;
    private double  initRotateVelocity     = 0;
    
    // Rate limiters for auto drive
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter rotateLimiter;

    // Auto drive to points X controller - need 2 controllers for X and Y for both setpoints
    private static final double adp = MAX_WHEEL_SPEED / 2; // 2 meter away --> full power
    private static final double adi = 0;
    private static final double add = 0;
    PIDController autoDriveXController;
    PIDController autoDriveYController;

    // Auto drive to points rotate controller
    private static final double adrp = MAX_ROTATE_SPEED * ((0.7) / Math.PI); // 1/0.7 Pi radians away --> full power
    private static final double adri = 0;
    private static final double adrd = 0;
    PIDController autoDriveRotateController;

    // Object Creation
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    public  SwerveDriveKinematics swerveDriveKinematics;

    // Rotate variables
    private int setpointCounter = 0;
    private boolean firstTime = true;
    private PIDController rotatePidController;
    private final double ROTATE_TOLERANCE_RADIANS = 0.05;
    private PIDController aprilTagRotatePidController;

    // NavX
    public static AHRS ahrs;

    /**
     * The constructor for the Drive class
     */
    public Drive() {
        // NavX
        try {
            ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            System.out.println("[ERROR] >> Error Instantiating navX MXP: " + ex.getMessage() + "\n");
        }

        ahrs.reset();

        while (ahrs.isConnected() == false) {
            // System.out.println("Connecting navX");
        }

        System.out.println("[INFO] >> navX Connected...");

        while (ahrs.isCalibrating() == true) {
            System.out.println("[INFO] >> Calibrating navX...");
        }

        System.out.println("[INFO] >> navX Ready");

        ahrs.zeroYaw();

        // Initializing rate limiters
        xLimiter = new SlewRateLimiter(24);
        yLimiter = new SlewRateLimiter(24);
        rotateLimiter = new SlewRateLimiter(8 * Math.PI);

        /* The locations for the modules must be relative to the center of the robot. 
         * Positive x values represent moving toward the front of the robot 
         * whereas positive y values represent moving toward the left of the robot 
         * Values are in meters
         */
        FRONT_LEFT_LOCATION  = new Translation2d(0.26035, 0.26035);
        FRONT_RIGHT_LOCATION = new Translation2d(0.26035, -0.26035);
        BACK_LEFT_LOCATION   = new Translation2d(-0.26035, 0.26035);
        BACK_RIGHT_LOCATION  = new Translation2d(-0.26035, -0.26035);

        // Creates the kinematics
        swerveDriveKinematics = new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            BACK_LEFT_LOCATION,
            BACK_RIGHT_LOCATION
        );

        // Creates the swerve modules. Encoders should be zeroed with the block
        frontLeft  = new SwerveModule(14, 15, true);
        frontRight = new SwerveModule(16, 17, false);
        backLeft   = new SwerveModule(12, 13, true);
        backRight  = new SwerveModule(10, 11, false);

        // PID instantiation
        autoDriveXController = new PIDController(adp, adi, add);
        autoDriveXController.setTolerance(AUTO_DRIVE_TOLERANCE);

        autoDriveYController = new PIDController(adp, adi, add);
        autoDriveYController.setTolerance(AUTO_DRIVE_TOLERANCE);

        autoDriveRotateController = new PIDController(adrp, adri, adrd);
        autoDriveRotateController.setTolerance(AUTO_DRIVE_ROTATE_TOLERANCE);
        autoDriveRotateController.enableContinuousInput(Math.PI, -Math.PI);
        
        rotatePidController = new PIDController(1.65, 0, 0);
        rotatePidController.setTolerance(ROTATE_TOLERANCE_RADIANS);
        rotatePidController.enableContinuousInput(Math.PI, -Math.PI);
        
        // TODO Get PID snappier, 0.80 has a little wobble but isnt that fast
        aprilTagRotatePidController = new PIDController(0.80, 0, 0);
        aprilTagRotatePidController.setTolerance(ROTATE_TOLERANCE_RADIANS);
        aprilTagRotatePidController.enableContinuousInput(Math.PI, -Math.PI);
    }

    /**
     * The function to drive the robot using a joystick.
     * <p>Positive Forward Goes Forward, Positive Strafe Goes Left, and Positive Rotation Speed is Clockwise 
     * @param forwardSpeed
     * @param strafeSpeed
     * @param rotationSpeed Positive is clockwise
     * @param fieldDrive
     */
    public void teleopDrive(double forwardSpeed, double strafeSpeed, double rotationSpeed, boolean fieldDrive) {
        // Calulates the SwerveModuleStates and determines if they are field relative
        // ChassisSpeeds uses positive values for going left(strafeSpeed)
        // ChassisSpeeds uses negative values for clockwise rotations(rotationSpeed)
        // Multiply by -1 for rotationSpeed to change input(positive for clockwise) to ChassisSpeeds
        SwerveModuleState[] swerveModuleStates = 
            swerveDriveKinematics.toSwerveModuleStates(
                fieldDrive
                ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, strafeSpeed * -1, rotationSpeed * -1, new Rotation2d( getYawAdjusted() ))
                : new ChassisSpeeds(forwardSpeed, strafeSpeed * -1, rotationSpeed * -1));

        // Limits the max speed of the wheels
        /* When adding the drive vector to the rotate vector the amplitude of the resulting vector
         * could be greater than 1.0 (max wheel speed).  When you desaturate you reduce the power of all
         * the wheels by the same amount so that no wheel power is saturated.
         */
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_WHEEL_SPEED);

        // Only moves wheels when given command
        if (Math.abs(forwardSpeed) > 0.04 || Math.abs(strafeSpeed) > 0.04 || Math.abs(rotationSpeed) > 0.04) {
            // The SwerveModuleStates array index used must match the order from the SwerveDriveKinematics instantiation
            frontLeft.setDesiredState(swerveModuleStates[0]);
            frontRight.setDesiredState(swerveModuleStates[1]);
            backLeft.setDesiredState(swerveModuleStates[2]);
            backRight.setDesiredState(swerveModuleStates[3]);
        }
        else {
            stopWheels();
        }
    }

    /**
     * Drives N feet
     * @param distance -> The distance to drive in feet
     * @param power -> The power to apply to the motor(-1 - 1)
     * @return
     */
    public int driveDistance(double distanceFeet, double power) {
        // If this function is being run for the first time, find the encoder 
        // tick value (Current encoder tick + Number of ticks in the distance parameter)
        if (driveDistanceFirstTime) {
            driveDistanceFirstTime = false;

            /*   TJM
             * I think this will cause problems in the future.
             * When you use odometry to get you position on the field is uses the encoders.
             * I suspect when we 0 out the encoder it will mess up the odometry.
             * I'd suggest creating an instance variable that contains the target distance and 
             * not zero the encoder.
             */
            backRight.zeroDriveEncoder();
            
            System.out.println("Current position: " + backRight.getDrivePositionFeet() + "ft");
            System.out.println("Target position: " + distanceFeet + "ft");
        }

        // If the robot has traveled the correct distance, stop the wheels and reset the drive distance
        if (backRight.getDrivePositionFeet() >= distanceFeet) {  
            System.out.println("Done, traveled " + backRight.getDrivePositionFeet() + "ft");
            driveDistanceFirstTime = true;
            stopWheels();
            return Robot.DONE;
        }      

        /* TJM
         *  If we set swerveModulesStates we can power the rotate motor in case it moves while driving
         * 
         * Do something like this for all 4 wheels
         * swerveModuleStates[0].speedMetersPerSecond = power
         * swerveModuleStates[0].angle = new Rotatation2d(angle)  use the same angle as we rotated to
         * 
         *  frontLeft.setDesiredState(swerveModuleStates[0]);
         *  frontRight.setDesiredState(swerveModuleStates[1]);
         *  backLeft.setDesiredState(swerveModuleStates[2]);
         *  backRight.setDesiredState(swerveModuleStates[3]);
         */
        // Sets drive power and module rotation in radians
        SwerveModuleState state = new SwerveModuleState(power, new Rotation2d(0.0));
        


        frontLeft.setDesiredState(state);
        frontRight.setDesiredState(state);
        backLeft.setDesiredState(state);
        backRight.setDesiredState(state);

        System.out.println(backRight.getDrivePositionFeet());
        return Robot.CONT;
    }

    /**
     * Automatically drives through a list of points.
     * @param listOfPoints
     * @param currPose
     * @return
     */
    public int autoDriveToPoints(Pose2d[] listOfPoints, Pose2d currPose) {
        // Grabs the target point
        Pose2d targetPoint = listOfPoints[autoPointIndex];

        // This runs once for each point in the list
        if (autoPointFirstTime == true) {
            autoPointFirstTime = false;
            autoDriveXController.reset();
            autoDriveYController.reset();
            autoDriveRotateController.reset();
            autoDriveXController.setSetpoint(targetPoint.getX());
            autoDriveYController.setSetpoint(targetPoint.getY());
            autoDriveRotateController.setSetpoint(targetPoint.getRotation().getRadians());
            autoPointAngled = false;

            // For each point except the last
            if (autoPointIndex < listOfPoints.length - 1) {
                autoDriveXController.setTolerance(2 * AUTO_DRIVE_TOLERANCE);
                autoDriveYController.setTolerance(2 * AUTO_DRIVE_TOLERANCE);
                autoDriveRotateController.setTolerance(2 * AUTO_DRIVE_ROTATE_TOLERANCE);
            }
            else {
                autoDriveXController.setTolerance(AUTO_DRIVE_TOLERANCE);
                autoDriveYController.setTolerance(AUTO_DRIVE_TOLERANCE);
                autoDriveRotateController.setTolerance(AUTO_DRIVE_ROTATE_TOLERANCE);
            }

            initXVelocity      = autoDriveXController.calculate(currPose.getX(), targetPoint.getX());
            initYVelocity      = autoDriveYController.calculate(currPose.getY(), targetPoint.getY());
            initRotateVelocity = autoDriveRotateController.calculate(currPose.getRotation().getRadians(), targetPoint.getRotation().getRadians());
        }
        // Runs when it's not the first time for a point
        else {
            // Angles the wheels if they are not aligned before driving
            if (autoPointAngled == false) {
                int rotateStatus = rotateWheels(initXVelocity, initYVelocity, initRotateVelocity);
                if (rotateStatus == Robot.DONE) {
                    autoPointAngled = true;
                }
            }
            // Drives normally once wheels are angled
            else {
                // Calculating targetVelocity based on distance to targetPoint
                double targetXVelocity      = autoDriveXController.calculate(currPose.getX(), targetPoint.getX());
                double targetYVelocity      = autoDriveYController.calculate(currPose.getY(), targetPoint.getY());
                double targetRotateVelocity = autoDriveRotateController.calculate(getYawAdjusted(), targetPoint.getRotation().getRadians());

                targetXVelocity = xLimiter.calculate(targetXVelocity);
                targetYVelocity = yLimiter.calculate(targetYVelocity);
                targetRotateVelocity = rotateLimiter.calculate(targetRotateVelocity);

                // Actual movement - only if wheels are rotated
                teleopDrive(targetXVelocity, targetYVelocity, targetRotateVelocity, true);
            }
        }

        // If X, Y, and Rotation are at target, moves on to next point
        if (autoDriveXController.atSetpoint() && autoDriveYController.atSetpoint() && autoDriveRotateController.atSetpoint()) {
            autoPointIndex++;
            autoPointFirstTime = true;
        }

        // Function ends once we pass the last point
        if (autoPointIndex >= listOfPoints.length) {
            autoPointIndex = 0;
            autoPointFirstTime = true;
            stopWheels();
            return Robot.DONE;
        }

        return Robot.CONT;
    }

    /*
     * Resets all instance variables used in driveToPoints
     */
    public void resetDriveToPoints() {
        autoPointFirstTime = true;
        autoPointIndex = 0;
    }

    /**
     * <p>Rotates wheels based on a drive command without giving the drive motors full power
     * <p>Uses wheel optimizations
     * @param driveX The x velocity
     * @param driveY The y velocity
     * @param driveZ The rotational velocity
     * @return Robot status, CONT or DONE
     */
    public int rotateWheels(double driveX, double driveY, double driveZ) {
        /* TJM
         * We are always using field oriented information even if we are not in field oriented mode.
         * See teleop drive...
         * 
         * I added notes in Robot where you were calling this....
         */
         SwerveModuleState[] swerveModuleStates = 
            swerveDriveKinematics.toSwerveModuleStates( ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, driveZ, new Rotation2d( getYawAdjusted() )));
        
        // Makes sure the wheels only rotate, not drive forward(zeros forward speed)
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 0);
        
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);

        if (frontLeft.rotateControllerAtSetpoint() && frontRight.rotateControllerAtSetpoint() &&
            backLeft.rotateControllerAtSetpoint() && backRight.rotateControllerAtSetpoint()) {
                return Robot.DONE;
        }

        return Robot.CONT;
    }

    /**
     * <p>Rotates wheels based on a drive command without giving the drive motors full power
     * <p>Does not use wheel optimizations
     * @param driveX
     * @param driveY
     * @param driveZ
     * @return Robot status, CONT or DONE
     */
    public int rotateWheelsNoOpt(double driveX, double driveY, double driveZ) {
        SwerveModuleState[] swerveModuleStates = 
            swerveDriveKinematics.toSwerveModuleStates( ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, driveZ, new Rotation2d( getYawAdjusted() )));
        
        // Makes sure the wheels only rotate, not drive forward(zeros forward speed)
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 0);
        
        // Do not use optimizate to make wheels rotate to absolute angle instead of optimized angle
        frontLeft.setDesiredStateNoOpt(swerveModuleStates[0]);
        frontRight.setDesiredStateNoOpt(swerveModuleStates[1]);
        backLeft.setDesiredStateNoOpt(swerveModuleStates[2]);
        backRight.setDesiredStateNoOpt(swerveModuleStates[3]);

        if (frontLeft.rotateControllerAtSetpoint() && frontRight.rotateControllerAtSetpoint() &&
            backLeft.rotateControllerAtSetpoint() && backRight.rotateControllerAtSetpoint()) {
                return Robot.DONE;
        }

        return Robot.CONT;
    }

    /**
     * <p>Rotates the robot in place to the desired angle
     * <p>Uses the PID controller for auto(autoDriveRotateController)
     * @param targetRadians The desired angle to rotate to
     * @return Robot status
     */
    public int rotateRobot(double targetRadians) {
        if(firstTime){
            // Reset control variables
            firstTime = false;
            setpointCounter = 0;

            // Reset PID Controller
            rotatePidController.setSetpoint(targetRadians);
        }
        double rotateVelocity = rotatePidController.calculate(getYawAdjusted(), targetRadians);
        
        // Increment setpointCounter if the robot is at the setpoint
        if(rotatePidController.atSetpoint()){
            setpointCounter++;
            // Robot has finished its rotation
            if(setpointCounter >= 5){
                firstTime = true;
                teleopDrive(0, 0, 0, false); // Stop rotating
                return Robot.DONE;
            }
        }
        else {
            setpointCounter = 0;    // Reset setpoint counter
        }
        
        // Get rotate velocity and rotate with teleopDrive()
        teleopDrive(0, 0, MathUtil.clamp(rotateVelocity, -1.0, 1.0), false);
        
        return Robot.CONT;
    }

    /**
     * <p>Sort of wrapper function which uses an AprilTag as a target angle
     * <p>If the pipeline is out of range, it returns Robot.FAIL
     * <p>The id param is only used to ensure the correct AprilTag is found
     * @param pipeline The limelight pipeline to searchfor the AprilTag
     * @param id The ID of the target AprilTag
     * @return Robot status
     */
    public int alignWithAprilTag(int pipeline, int id) {
        // Get the NetworkTable for the limelight
        NetworkTable aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight");

        if(firstTime){
            aprilTagTable.getEntry("pipeline").setNumber(pipeline); // Set the pipeline
            setpointCounter = 0;
            aprilTagRotatePidController.setSetpoint(0); // Reset PID Controller
            firstTime = false;
        }
        
        /* Check if there's a valid AprilTag in vision and
         * if the target AprilTag is the one we're looking for
         *  tv is a double for if an AprilTag is in view
         *  tid is a double for the id of the target AprilTag
         */
        if(aprilTagTable.getEntry("tv").getDouble(0.0) == 1 &&
            aprilTagTable.getEntry("tid").getDouble(0.0) == id){
            
            // Get the horizontal offset of the AprilTag to the crosshair, convert to radians
            double targetOffset = Math.toRadians(aprilTagTable.getEntry("tx").getDouble(0.0));
            double rotateVelocity = -1 * aprilTagRotatePidController.calculate(targetOffset, 0);

            // Increment setpointCounter if the robot is at the setpoint
            if(aprilTagRotatePidController.atSetpoint()){
                setpointCounter++;
                // Robot has finished its rotation
                if(setpointCounter >= 5){
                    firstTime = true;
                    teleopDrive(0, 0, 0, false); // Stop rotating
                    return Robot.DONE;
                }
            }
            else {
                setpointCounter = 0;    // Reset setpoint counter
            }
            
            // Get rotate velocity and rotate with teleopDrive()
            teleopDrive(0, 0, MathUtil.clamp(rotateVelocity, -1.0, 1.0), false);

            return Robot.CONT;
        }
        return Robot.FAIL;  // No AprilTag in sight, fail
    }

    /******************************************************************************************
    *
    *    SETTING FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * <p>Sets the swerve ModuleStates.
     * <p>Uses optimizations
     * @param desiredStates
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Limits the wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, POWER_SPEED_RATIO_MPS);

        // Sets the desired states
        frontLeft .setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft  .setDesiredState(desiredStates[2]);
        backRight .setDesiredState(desiredStates[3]);
    }

    /****************************************************************************************** 
    *
    *    HELPER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Stops the wheels.
     */
    public void stopWheels() {
        frontLeft.setDriveMotorPower(0.0);
        frontLeft.setRotateMotorPower(0.0);
        frontRight.setDriveMotorPower(0.0);
        frontRight.setRotateMotorPower(0.0);
        backLeft.setDriveMotorPower(0.0);
        backLeft.setRotateMotorPower(0.0);
        backRight.setDriveMotorPower(0.0);
        backRight.setRotateMotorPower(0.0);
    }

    /**
     * Resets the Yaw on the NavX.
     */
    public void resetYaw() {
        ahrs.zeroYaw();
    }

    public boolean gyroConnected() {
        return ahrs.isConnected();
    }

    /****************************************************************************************** 
    *
    *    GETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Gets the Front Left Module's position.
     * 
     * @return The FrontLeft Module Position
     */
    public SwerveModulePosition getFLPosition() {
        return frontLeft.getModulePosition();
    }

    /**
     * Gets the Front Right Module's position.
     * 
     * @return The FrontRight Module Position
     */
    public SwerveModulePosition getFRPosition() {
        return frontRight.getModulePosition();
    }

    /**
     * Gets the Back Left Module's position.
     * 
     * @return The BackLeft Module Position
     */
    public SwerveModulePosition getBLPosition() {
        return backLeft.getModulePosition();
    }

    /**
     * Gets the Back Right Module's position.
     * 
     * @return The BackRight Module Position
     */
    public SwerveModulePosition getBRPosition() {
        return backRight.getModulePosition();
    }

    /**
     * Converts degrees to radians from NavX
     * 
     */
    public double getYawAdjusted() {
        return MathUtil.angleModulus(Units.degreesToRadians( ahrs.getYaw() ));
    }

    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/  
    /**
     * Inits the motor sliders
     */
    public void initWheelPowerTests() {
        frontLeft.initMotorSliders();
        frontRight.initMotorSliders();
        backLeft.initMotorSliders();
        backRight.initMotorSliders();
    }

    /**
     * Tests the wheel powers
     */
    public void testWheelPowers() {
        frontLeft.updateMotorPowers();
        frontRight.updateMotorPowers();
        backLeft.updateMotorPowers();
        backRight.updateMotorPowers();
    }

    /**
     * Zeros the motor encoders
     */
    public void zeroDriveEncoders() {
        frontLeft .zeroDriveEncoder();
        frontRight.zeroDriveEncoder();
        backLeft  .zeroDriveEncoder();
        backRight .zeroDriveEncoder();
    }

    /**
     * Displays the enocder values
     */
    public void testEncoders() {
        frontLeft .displayEncoderValues();
        frontRight.displayEncoderValues();
        backLeft  .displayEncoderValues();
        backRight .displayEncoderValues();
    }

    /**
     * Tests the modules by rotating them to different positions
     * 
     * @param radians
     */
    public void testModuleRotation(double radians) {
        // Creates the target positions
        SwerveModuleState   targetState      = new SwerveModuleState(0, new Rotation2d(radians));
        SwerveModuleState[] targetStateArray = {targetState, targetState, targetState, targetState};

        // Updates the encoders
        testEncoders();

        // Forces the wheels to move to it
        setModuleStates(targetStateArray);

        // Updates the encoders
        testEncoders();
    }

    /**
     * 
     */
    public void printPowerandVelocity() {
        if (printCount % 15 == 0) {
            frontLeft.displayPowerAndVelocity();
            frontRight.displayPowerAndVelocity();
            backLeft.displayPowerAndVelocity();
            backRight.displayPowerAndVelocity();
        }
        printCount++;
    }

    /**
     * 
     */
    public void testGyro() {
        if (printCount % 15 == 0) {
            System.out.println("Adjusted angle: " + getYawAdjusted());
        }
        printCount++;
    }

    /**
     * 
     */
    public void periodicTestDrivePower() {
        double drivePower = SmartDashboard.getNumber("Drive Power", 0);
        teleopDrive(drivePower, 0, 0, false);

        if (printCount % 5 == 0) {
            System.out.println("Drive Speed: " + frontLeft.getDriveVelocity());
        }
        printCount++;
    }

    /**
     * Sets all drive motors to the given power
     * @param power
     */
    public void setAllDriveMotorPower(double power) {
        frontLeft.setDriveMotorPower(power);
        frontRight.setDriveMotorPower(power);
        backLeft.setDriveMotorPower(power);
        backRight.setDriveMotorPower(power);
    }

	/**
	 * <p> Checks if all swerve module rotate encoders at setpoint
     * 
	 * @return Whether all of the swerve modules at setpoint
	 */
	public boolean allEncodersAtSetpoint() {
		// Test if all angles are within rotate tolerance
		return 
			frontLeft.rotateControllerAtSetpoint() && 
			frontRight.rotateControllerAtSetpoint() && 
			backLeft.rotateControllerAtSetpoint() && 
			backRight.rotateControllerAtSetpoint();
	}

    public void testAprilTagID() {
        NetworkTable aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight");

        double tid = aprilTagTable.getEntry("tid").getDouble(0.0);
        double tv = aprilTagTable.getEntry("tv").getDouble(0);

        SmartDashboard.putNumber("LimelightID", tid);
        SmartDashboard.putNumber("LimelightValidTag", tv);
    }

    public void testAprilTagXY() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        /*
         * tx:
         *  Horizontal offset from crosshair to target
         *  Positive is to the right relative to the camera
         *  LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees
         * ty:
         *  Vertical offset from crosshair to target
         *  Positive is downwards relative to the camera
         *  LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees
         * ta:
         *  Target area percent, 0% - 100% of image
         */
        double tx = table.getEntry("tx").getDouble(0.0);    // Target x
        double ty = table.getEntry("ty").getDouble(0.0);    // Target y
        double ta = table.getEntry("ta").getDouble(0.0);    // Target area

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
    }

    public void testAprilTagPipeline(int pipeline) {
        NetworkTable aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        aprilTagTable.getEntry("pipeline").setNumber(pipeline); // Set the new pipeline
        
        double pipelineNT = aprilTagTable.getEntry("pipeline").getDouble(0);    // Check if the change is reflected in NetworkTables
        SmartDashboard.putNumber("LimelightPipeline", pipelineNT);  // Display current pipeline from NetworkTables
    }


}

// End of the Drive class
