package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;

public class Arm {
    private final int EXTENDER_MOTOR_CAN = 49; // 3
    private final int ELEVATION_MOTOR_CAN = 23;
    private final int MOTOR_CURRENT_LIMIT = 40;
    private final int INTAKE1_BUTTON_ID = 2;
    private final int INTAKE2_BUTTON_ID = 3;
    private final int REST_BUTTON_ID = 4;
    
    private final double LOWER_EXTENSION_LIMIT_MM = 58;
    private final double UPPER_EXTENSION_LIMIT_MM = 155;
    private final double LOWER_ELEVATION_LIMIT = 0;
    private final double UPPER_ELEVATION_LIMIT = Math.PI;
    
    private final double EXTENSION_TOLERANCE_MM = 10;    // TODO Test and tune
    private final double ELEVATION_TOLERANCE_DEGREES = 2;
    
    private final double ELEVATION_ENCODER_FACTOR = 360;
    
    private final double ARM_ELEVATION_PID_P = 0.04;
    private final double ARM_ELEVATION_PID_I = ARM_ELEVATION_PID_P / 1.5;
        
    public final double ARM_REST_POSITION_DEGREES = 329;
    public final double ARM_AMP_POSITION_DEGREES = 25.65;

    private CANSparkMax extenderMotor;
    private CANSparkMax elevationMotor;
    
    private AbsoluteEncoder elevationEncoder;
    
    private PIDController extenderMotorPidController;
    private PIDController elevationMotorPidController;
    
    private DigitalInput restStopButton;
    private DigitalInput intake1StopButton;
    private DigitalInput intake2StopButton;
    
    private int rotateTargetCount = 0;
    
    private AnalogPotentiometer extenderPot;
    private final int EXTENDER_POTENTIOMETER_ID = 0;
    private final double EXTENDER_POTENTIOMETER_ZERO = -3;     // ~3mm offset to zero
    private final double EXTENDER_POTENTIOMETER_RANGE = 1000;  // Max value, in mm
    //public final double ARM_POT_REST_POSITION = 58;
    //public final double ARM_POT_INTAKE_POSITION = 135;

    // Action variables
    private boolean extensionFirstTime;
    private boolean retract = false;
    private double extensionDistance;

    private boolean elevationFirstTime;
    private double elevationAngle;

    private double startPosition;
    private double targetDistance;

    private double rotateStatus = Robot.CONT;
    private double extendStatus = Robot.CONT;

    public Arm() {        
        //System.out.println("[INFO] >> Initializing arm motors...");

        // Setup extender motor
        extenderMotor = new CANSparkMax(EXTENDER_MOTOR_CAN, MotorType.kBrushless);
        extenderMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        extenderMotor.setIdleMode(IdleMode.kBrake);
        extenderMotor.setInverted(false);
        
        // Setup elevation motor
        elevationMotor = new CANSparkMax(ELEVATION_MOTOR_CAN, MotorType.kBrushless);
        elevationMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        elevationMotor.setIdleMode(IdleMode.kBrake);

        // Encoders        
        //System.out.println("[INFO] >> Initializing arm encoder(s)...");
        
        // Extender Potentiometer
        extenderPot = new AnalogPotentiometer(
            EXTENDER_POTENTIOMETER_ID, 
            EXTENDER_POTENTIOMETER_RANGE, 
            EXTENDER_POTENTIOMETER_ZERO
        );

        // Extender Retraction Button
        restStopButton = new DigitalInput(REST_BUTTON_ID);
        intake1StopButton = new DigitalInput(INTAKE1_BUTTON_ID);
        intake2StopButton = new DigitalInput(INTAKE2_BUTTON_ID);

        // Elevation Absolute
        elevationEncoder = elevationMotor.getAbsoluteEncoder(Type.kDutyCycle);
        elevationEncoder.setPositionConversionFactor(ELEVATION_ENCODER_FACTOR);
        elevationEncoder.setVelocityConversionFactor(ELEVATION_ENCODER_FACTOR);

        // PID controllers
        //System.out.println("[INFO] >> Initializing arm PID controllers...");
        
        // Extender PID
        extenderMotorPidController = new PIDController(1.0, 0.0, 0.0);
        extenderMotorPidController.setTolerance(EXTENSION_TOLERANCE_MM);

        // Elevation PID
        elevationMotorPidController = new PIDController(ARM_ELEVATION_PID_P, ARM_ELEVATION_PID_I, 0.0);
        elevationMotorPidController.setIntegratorRange(-0.1, 0.1);
        elevationMotorPidController.setTolerance(ELEVATION_TOLERANCE_DEGREES);
        elevationMotorPidController.enableContinuousInput(0, 360);
        
        // Action variables
        extensionFirstTime = true;
        extensionDistance = 0.0;

        elevationFirstTime = true;
        elevationAngle = 0.0;

        startPosition = elevationEncoder.getPosition();
        //System.out.println("[INFO] >> Arm start position: " + startPosition);
    }

    /**
     * Extends the arm to the given distance with a PID controller
     * @param distance The distance to extend to, in mm
     * @return Robot.CONT or Robot.DONE
     */
    public int extendArmWithPID(double distance) {
        distance = MathUtil.clamp(distance, LOWER_EXTENSION_LIMIT_MM, UPPER_EXTENSION_LIMIT_MM);  // Limit extension

        if(extensionFirstTime) {
            extensionFirstTime = false;
        }

        double power = extenderMotorPidController.calculate(getExtendPosition(), distance);
        extenderMotor.set(MathUtil.clamp(power, -1, 1));

        if(extenderMotorPidController.atSetpoint()) {
            extensionFirstTime = true;

            return Robot.DONE;
        }
        
        return Robot.CONT;
    }

    /**
     * Extends the arm to the given distance
     * @param distance The distance to extend to, in mm
     * @param power The power to extend with (positive power retracts, negative power extends)
     * @return Robot.CONT or Robot.DONE
     */
    public int extendArm(double distance, double power) {
        distance = MathUtil.clamp(distance, LOWER_EXTENSION_LIMIT_MM, UPPER_EXTENSION_LIMIT_MM);  // Limit extension

        if(extensionFirstTime) {
            extensionFirstTime = false;
        }
        
        extenderMotor.set(MathUtil.clamp(power, -1, 1));
        
        if(getExtendPosition() >= targetDistance) {
            extensionFirstTime = true;
            extenderMotor.set(0);

            return Robot.DONE;
        }
        
        return Robot.CONT;
    }

    /**
     * Extends the arm to the given distance
     * @param targetPosition The position to extend to, in mm
     * @param power The power to extend with (positive power retracts, negative power extends)
     * @return Robot.CONT or Robot.DONE
     */
    private int extendArmToPosition(double targetPosition, double power) {
        double currentPosition = getExtendPosition();

        if(extensionFirstTime) {
            extensionFirstTime = false;

            if(currentPosition > targetPosition) {
                retract = true;
            }
            else if (currentPosition < targetPosition) {
                retract = false;
            }
            else {
                extensionFirstTime = true;
                extenderMotor.set(0);
                return Robot.DONE;
            }
        }

        targetPosition = MathUtil.clamp(targetPosition, LOWER_EXTENSION_LIMIT_MM, UPPER_EXTENSION_LIMIT_MM);  // Limit extension
        
        // Retract the arm if it's too far out
        if (retract == true) {
            extenderMotor.set(MathUtil.clamp(power, -1, 1));

            if (currentPosition <= targetPosition) {
                extensionFirstTime = true;
                extenderMotor.set(0);
                return Robot.DONE;
            }
        }
        else {
            extenderMotor.set(MathUtil.clamp(power * -1, -1, 1));

            if (currentPosition >= targetPosition) {
                extensionFirstTime = true;
                extenderMotor.set(0);
                return Robot.DONE;
            }
        }
        
        return Robot.CONT;
    }

    /**
     * Sets motor power to 0.4 until the button is activated
     * @return CONT, if the button isn't activated
     * <p>     DONE, if the button is activated
     */
    public int extendToRest() {
        if(restStopButton.get()){
            extenderMotor.set(0.4);
            
            return Robot.CONT;
        }
        else {
            extenderMotor.set(0);
            return Robot.DONE;
        }

        //return extendArmToPosition(ARM_POT_REST_POSITION, 0.4);
    }

    /**
     * Sets motor power to -0.4
     * @return CONT
     * <p>     DONE
     */
    public int extendToIntake() {
        if(intake1StopButton.get() || intake2StopButton.get()){
            extenderMotor.set(-0.4);
            
            return Robot.CONT;
        }
        else {
            extenderMotor.set(0);
            
            return Robot.DONE;
        }
        //return extendArmToPosition(ARM_POT_INTAKE_POSITION, 0.4);
    }

    /**
     * <p>Rotates the arm with the given radian value
     * <p>0 degrees is horizontal
     * 
     * @param degrees The angle to rotate to in degrees
     * @return Robot.CONT or Robot.DONE
     */
    public int rotateArm(double degrees) {
        if(elevationFirstTime) {
            elevationFirstTime = false;
            elevationMotorPidController.setSetpoint(degrees);
            rotateTargetCount = 0;
        }

        //System.out.println("From: " + elevationEncoder.getPosition() + " To: " + degrees);
        /* Negative power moves the arm upward;
            The PID value will be positive to increase the angle */
        double power = -elevationMotorPidController.calculate(elevationEncoder.getPosition(), degrees);
        //SmartDashboard.putNumber("Arm power", power);
        elevationMotor.set(MathUtil.clamp(power, -0.3, 0.3)); // Clamp
                
        if(elevationMotorPidController.atSetpoint()) {
            rotateTargetCount++;

            if (rotateTargetCount >= 5) {
                elevationFirstTime = true;
                //elevationMotor.set(0.1);
                return Robot.DONE;
            }
        }
        else{
            rotateTargetCount = 0;
        }

        return Robot.CONT;
    }

    public int rotateToRest(double powerMultiplier) {
        if(elevationFirstTime) {
            elevationFirstTime = false;
            elevationMotorPidController.setSetpoint(ARM_REST_POSITION_DEGREES);
            rotateTargetCount = 0;
        }

        //System.out.println("From: " + elevationEncoder.getPosition() + " To: " + degrees);
        /* Negative power moves the arm upward;
            The PID value will be positive to increase the angle */
        double power = -elevationMotorPidController.calculate(elevationEncoder.getPosition(), ARM_REST_POSITION_DEGREES);
        power *= powerMultiplier;
        //SmartDashboard.putNumber("Arm power", power);
        elevationMotor.set(MathUtil.clamp(power, -0.3, 0.3)); // Clamp
                
        if(elevationMotorPidController.atSetpoint()) {
            rotateTargetCount++;

            if (rotateTargetCount >= 5) {
                elevationFirstTime = true;
                //elevationMotor.set(0.1);
                return Robot.DONE;
            }
        }
        else{
            rotateTargetCount = 0;
        }

        return Robot.CONT;
    }

    /**
     * <p>Keeps the arm at its current position
     * <p>0 degrees is horizontal
     * 
     * @param degrees The angle to rotate to in degrees
     * @return Robot.CONT or Robot.DONE
     */
    public int maintainPosition(double degrees) {                
        /* Negative power moves the arm upward;
            The PID value will be positive to increase the angle */
        double power = -elevationMotorPidController.calculate(elevationEncoder.getPosition(), degrees);
        elevationMotor.set(MathUtil.clamp(power, -0.4, 0.4));   // Clamp                
        return Robot.CONT;
    }

    /**************************************************************************
     * 
     *      ACTION FUNCTIONS
     * 
     **************************************************************************/

    /**
     * Rotates arm incrementaly up
     * 
     * @param rotateUp
     * @param rotateDown
     */
    public void rotateArmIncrement(boolean rotateUp, boolean rotateDown) {
        if(rotateUp){
            elevationAngle += 0.75;
        }
        else if(rotateDown) {
            elevationAngle -= 0.75;
        } 
        else {
            elevationAngle += 0;
        }

        elevationAngle = MathUtil.clamp(elevationAngle, LOWER_ELEVATION_LIMIT, UPPER_ELEVATION_LIMIT);

        rotateArm(elevationAngle);
    }

    /**
     * Extends arm incrementaly, 5mm steps
     * 
     * @param extend
     * @param retract
     */
    public void moveArmIncrement(boolean extend, boolean retract) {
        if(extend) {
            extensionDistance += 5;
        }
        else if(retract){
            extensionDistance -= 5;
        }
        else {
            extensionDistance += 0;
        }

        extensionDistance = MathUtil.clamp(extensionDistance, LOWER_EXTENSION_LIMIT_MM, UPPER_EXTENSION_LIMIT_MM);

        extendArm(extensionDistance, 0.1);
    }

    public double getElevationPosition() {
        return elevationEncoder.getPosition();
    }

    /**
     * Gets the extender position with the string potentiometer
     * @return The extender position
     */
    public double getExtendPosition() {
        return extenderPot.get();
    }

    /**
     * Turns off the elevation motor to let it fall
     */
    public void disableRotation() {
        elevationMotor.set(0);
    }


    /**************************************************************************
     * 
     *      TEST FUNCTIONS
     * 
     **************************************************************************/

    /// Positive power goes down
    public void testElevate(double power) {
        elevationMotor.set(power);
    }

    /// Put the arm position on shuffleboard
    public void testPosition() {
        System.out.println(elevationEncoder.getPosition());
    }

    /// Positive power retracts the motor
    public void testExtend(double power) {
        extenderMotor.set(power);
    }

    /// Move the arm to its starting/balancing position (54 degrees)
    public int testStartingPosition() {
        return rotateArm(54);
    }

    /// Returns the status of intake button 2
    public boolean getIntakeButton1() {
        return !intake1StopButton.get();
    }

    /// Returns the status of intake button 2
    public boolean getIntakeButton2() {
        return !intake2StopButton.get();
    }

    /// Returns the status of the intake buttons
    /// Returns whether either are pressed
    public boolean getIntakeButtons() {
        return !intake1StopButton.get() || !intake2StopButton.get();
    }
    
    /// Returns the status of the intake button
    public boolean getRestButton() {
        return !restStopButton.get();
    }
}
