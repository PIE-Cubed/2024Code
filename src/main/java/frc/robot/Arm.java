package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Arm {
    private final int EXTENDER_MOTOR_CAN = 3;
    private final int ELEVATION_MOTOR_CAN = 23;
    private final int MOTOR_CURRENT_LIMIT = 70;

    // TODO Find and tune limits
    private final double LOWER_EXTENSION_LIMIT = 0;
    private final double UPPER_EXTENSION_LIMIT = 0;
    private final double LOWER_ELEVATION_LIMIT = 0;
    private final double UPPER_ELEVATION_LIMIT = Math.PI;

    // TODO Tune tolerances
    private final double EXTENSION_TOLERANCE_TICKS = 1;    // Currently ticks until conversion factor is made
    private final double ELEVATION_TOLERANCE_DEGREES = 2;

    private final double EXTENDER_ENCODER_FACTOR = 1;  // TODO Find conversion factor
    private final double ELEVATION_ENCODER_FACTOR = 360;

    private final double ARM_EXTENDER_PID_P = 0.035;
    private final double ARM_EXTENDER_PID_I = ARM_EXTENDER_PID_P / 1.5;

    public final double ARM_REST_POSITION_DEGREES = 329;
    public final double ARM_AMP_POSITION_DEGREES = 33;


    private CANSparkMax extenderMotor;
    private CANSparkMax elevationMotor;
    
    private AbsoluteEncoder elevationEncoder;
    private RelativeEncoder extenderEncoder;
    
    private PIDController extenderMotorPidController;
    private PIDController elevationMotorPidController;

    // Action variables
    private boolean extensionFirstTime;
    private double extensionDistance;

    private boolean elevationFirstTime;
    private double elevationAngle;

    private double startPosition;

    public Arm() {
        // Setup extender motor
        extenderMotor = new CANSparkMax(EXTENDER_MOTOR_CAN, MotorType.kBrushless);
        extenderMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        extenderMotor.setIdleMode(IdleMode.kBrake);
        
        // Setup elevation motor
        elevationMotor = new CANSparkMax(ELEVATION_MOTOR_CAN, MotorType.kBrushless);
        elevationMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        elevationMotor.setIdleMode(IdleMode.kBrake);

        // Encoders
        extenderEncoder = extenderMotor.getEncoder();
        extenderEncoder.setPositionConversionFactor(EXTENDER_ENCODER_FACTOR);

        elevationEncoder = elevationMotor.getAbsoluteEncoder(Type.kDutyCycle);
        elevationEncoder.setPositionConversionFactor(ELEVATION_ENCODER_FACTOR);
        elevationEncoder.setVelocityConversionFactor(ELEVATION_ENCODER_FACTOR);

        // PID controllers
        // Extender PID
        extenderMotorPidController = new PIDController(1.0, 0.0, 0.0);
        extenderMotorPidController.setTolerance(EXTENSION_TOLERANCE_TICKS);

        // Elevation PID
        elevationMotorPidController = new PIDController(ARM_EXTENDER_PID_P, ARM_EXTENDER_PID_I, 0.0);
        elevationMotorPidController.setIntegratorRange(-0.1, 0.1);
        elevationMotorPidController.setTolerance(ELEVATION_TOLERANCE_DEGREES);
        elevationMotorPidController.enableContinuousInput(0, 360);
        

        // Action variables
        extensionFirstTime = true;
        extensionDistance = 0.0;

        elevationFirstTime = true;
        elevationAngle = 0.0;

        startPosition = elevationEncoder.getPosition();
        System.out.println("[INFO] >> Arm start position: " + startPosition);
    }

    /**
     * Extends the arm to the given distance with a PID
     * 
     * @param distance The distance to extend to, in encoder ticks
     * @return Robot.CONT or Robot.DONE
     */
    public int extendArmWintPID(double distance) {
        //distance = MathUtil.clamp(distance, LOWER_EXTENSION_LIMIT, UPPER_EXTENSION_LIMIT);  // Limit extension

        if(extensionFirstTime) {
            extensionFirstTime = false;
            extenderEncoder.setPosition(0);
            extenderMotorPidController.setSetpoint(distance);
        }

        double power = extenderMotorPidController.calculate(extenderEncoder.getPosition());
        extenderMotor.set(MathUtil.clamp(power, -1, 1));

        if(extenderMotorPidController.atSetpoint()) {
            extensionFirstTime = true;
            return Robot.DONE;
        }
        else {
            return Robot.CONT;
        }
    }

    /**
     * Extends the arm to the given distance
     * 
     * @param distance The distance to extend to, in encoder ticks
     * @return Robot.CONT or Robot.DONE
     */
    public int extendArm(double distance, double power) {
        //distance = MathUtil.clamp(distance, LOWER_EXTENSION_LIMIT, UPPER_EXTENSION_LIMIT);  // Limit extension

        if(extensionFirstTime) {
            extensionFirstTime = false;
            //extenderEncoder.setPosition(0);
            distance = extenderEncoder.getPosition() + distance;
            extenderMotor.set(MathUtil.clamp(power, -1, 1));
        }

        if(extenderEncoder.getPosition() >= distance) {
            extensionFirstTime = true;
            extenderMotor.set(0);
            return Robot.DONE;
        }

        return Robot.CONT;
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
        }

        //System.out.println("From: " + elevationEncoder.getPosition() + " To: " + degrees);
        /* Negative power moves the arm upward;
            The PID value will be positive to increase the angle */
        double power = -elevationMotorPidController.calculate(elevationEncoder.getPosition(), degrees);
        //SmartDashboard.putNumber("Arm power", power);
        elevationMotor.set(MathUtil.clamp(power, -0.2, 0.2));   // Clamp
                
        if(elevationMotorPidController.atSetpoint()) {
            elevationFirstTime = true;
            elevationMotor.set(0.1);
            return Robot.DONE;
        }
        else{
            return Robot.CONT;
        }
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
     * Extends arm incrementaly up
     * 
     * @param extend
     * @param retract
     */
    public void moveArmIncrement(boolean extend, boolean retract) {
        if(extend) {
            extensionDistance += 0.01;
        }
        else if(retract){
            extensionDistance -= 0.01;
        }
        else {
            extensionDistance += 0;
        }

        extensionDistance = MathUtil.clamp(extensionDistance, LOWER_EXTENSION_LIMIT, UPPER_EXTENSION_LIMIT);

        extendArm(extensionDistance, 0.1);
    }

    public double getElevationPosition() {
        return elevationEncoder.getPosition();
    }

    public double getExtendPosition() {
        return extenderEncoder.getPosition();
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

    /// Positive power extends the motor
    public void testExtend(double power) {
        extenderMotor.set(power);
    }

    /// Move the arm to its starting/balancing position (54 degrees)
    public int testStartingPosition() {
        return rotateArm(54);
    }
}
