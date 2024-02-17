package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.counter.ExternalDirectionCounter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Arm {
    private final int EXTENDER_MOTOR_CAN = 0;
    private final int ELEVATION_MOTOR_CAN = 0;
    private final int MOTOR_CURRENT_LIMIT = 70;

    private final double LOWER_EXTENSION_LIMIT = 0;
    private final double UPPER_EXTENSION_LIMIT = 0;
    private final double LOWER_ELEVATION_LIMIT = 0;
    private final double UPPER_ELEVATION_LIMIT = 0;

    private final double EXTENSION_TOLERANCE_METERS = 0.001;
    private final double ELEVATION_TOLERANCE_RADIANS = 0.05;

    private final double EXTENDER_ENCODER_FACTOR = 1.0;
    private final double ELEVATION_ENCODER_FACTOR = 2 * Math.PI;

    private CANSparkMax extenderMotor;
    private CANSparkMax elevationMotor;
    
    private AbsoluteEncoder elevationEncoder;
    private RelativeEncoder extenderEncoder;
    
    private PIDController extenderMotorPidController;
    private PIDController elevationMotorPidController;

    // Action variables
    private boolean extensionFirstTime;
    private int extensionSetpointCounter;
    private double extensionDistance;

    private boolean elevationFirstTime;
    private int elevationSetpointCounter;
    private double elevationAngle;

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

        // PID controllers
        extenderMotorPidController = new PIDController(1.0, 0.0, 0.0);
        extenderMotorPidController.setTolerance(EXTENSION_TOLERANCE_METERS);

        elevationMotorPidController = new PIDController(1.0, 0.0, 0.0);
        extenderMotorPidController.setTolerance(ELEVATION_TOLERANCE_RADIANS);

        // Action variables
        extensionFirstTime = true;
        extensionSetpointCounter = 0;
        extensionDistance = 0.0;

        elevationFirstTime = true;
        elevationSetpointCounter = 0;
        elevationAngle = 0.0;
    }

    /**
     * Extends the arm to the given distance
     * 
     * @param distance
     * @return Robot.CONT or Robot.DONE
     */
    public int extendArm(double distance) {
        if(extensionFirstTime) {
            extensionFirstTime = false;
            extensionSetpointCounter = 0;
        }

        if(extenderMotorPidController.atSetpoint()) {
            extensionSetpointCounter++;
            if(extensionSetpointCounter >= 5) {
                return Robot.DONE;
            }
        } else {
            extensionSetpointCounter = 0;
        }

        distance = MathUtil.clamp(distance, 0.0, LOWER_EXTENSION_LIMIT);  // Limit extension
        
        double power = extenderMotorPidController.calculate(extenderEncoder.getPosition(), distance);
        extenderMotor.set(MathUtil.clamp(power, -1, 1));

        return Robot.CONT;
    }

    /**
     * <p>Rotates the arm with the given radian value
     * <p>0 radians is horizontal
     * 
     * @param radians The angle to rotate to in radians
     * @return Robot.CONT or Robot.DONE
     */
    public int rotateArm(double radians) {
        if(elevationFirstTime) {
            elevationFirstTime = false;
            elevationSetpointCounter = 0;
        }
        if(elevationMotorPidController.atSetpoint()) {
            elevationSetpointCounter++;
            if(elevationSetpointCounter >= 5) {
                return Robot.DONE;
            }
        } else {
            elevationSetpointCounter = 0;
        }
        radians = MathUtil.clamp(radians, LOWER_ELEVATION_LIMIT, UPPER_ELEVATION_LIMIT);    // Limit rotation

        double power = elevationMotorPidController.calculate(elevationEncoder.getPosition(), radians);
        elevationMotor.set(MathUtil.clamp(power, -1, 1));
        
        return Robot.CONT;
    }

    /**
     * <p>Rotates the arm to -27 degrees, from horizontal
     * <p>Retracts the arm fully
     * 
     * @return Robot.CONT or Robot.DONE
     */
    public int returnToRestingPosition() {
        int rotateStatus = rotateArm(Math.toRadians(27));
        int extensionStatus = extendArm(0.0);

        // Only return Robot.CONT when both are done
        if(rotateStatus == Robot.DONE && extensionStatus == Robot.DONE) {
            return Robot.DONE;
        }
        else {
            return Robot.CONT;
        }
    }

    /**
     * Rotates arm incrementaly up
     */
    public void rotateArmIncrement(boolean rotateDown, boolean rotateUp) {
        if(rotateDown) {
            elevationAngle -= 0.05;
        }
        else if(rotateUp){
            elevationAngle += 0.05;
        }
        else {
            elevationAngle += 0;
        }

        elevationAngle = MathUtil.clamp(elevationAngle, LOWER_ELEVATION_LIMIT, UPPER_ELEVATION_LIMIT);

        rotateArm(elevationAngle);
    }

    /**
     * Extends arm incrementaly up
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

        extendArm(extensionDistance);
    }

}
