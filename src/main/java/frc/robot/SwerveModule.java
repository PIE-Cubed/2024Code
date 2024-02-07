// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Start of the SwerveModule class
 */
public class SwerveModule {
    // Variables
    private double prevPower = 0;

    // Constants
    private final int MOTOR_CURRENT_LIMIT = 80;
    
    // Controller Parameters
    private final double ROTATE_P = 0.2;
    private final double ROTATE_I = 0;
    private final double ROTATE_D = 0;

    // Drive Motor Conversion Factors
    private final double WHEEL_DIAMETER_METERS       = Units.inchesToMeters(2.875);
    private final double WHEEL_ROTATION_METERS       = Math.PI * WHEEL_DIAMETER_METERS;
    private final double ROTATIONS_PER_TICK          = 1 / 4.70 / 1; // 1 / external gearing / gearbox
    private final double DRIVE_POS_CONVERSION_FACTOR = WHEEL_ROTATION_METERS * ROTATIONS_PER_TICK; // Meters per tick
    private final double DRIVE_VEL_CONVERSION_FACTOR = DRIVE_POS_CONVERSION_FACTOR / 60;           // Meters per second

    // Absolute Encoder Conversion Factors
    private final double MODULE_POS_CONVERSION_FACTOR = 2 * Math.PI;                  // Radians per tick
    private final double MODULE_VEL_CONVERSION_FACTOR = MODULE_POS_CONVERSION_FACTOR; // Radians per second

    // Motor Creation
    private CANSparkFlex    driveMotor;
    private CANSparkMax     rotateMotor;

    // Encoder Creation
    private RelativeEncoder driveEncoder;
    private AbsoluteEncoder absoluteEncoder;

    // Object Creation
    private PIDController   rotateMotorController;

    /**
     * The constructor for the SwerveModule class
     *
     * @param driveID The CAN ID of the drive motor.
     * @param rotateMotorId The CAN ID of the rotate motor.
     * @param invertMotor Wheter the drive motor is reversed.
     */
    public SwerveModule(int driveID, int rotateID, boolean invertDriveMotor) {
        // Creates the drive motor
        driveMotor = new CANSparkFlex(driveID, MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(invertDriveMotor);

        // Creates the rotation motor
        rotateMotor = new CANSparkMax(rotateID, MotorType.kBrushless);
        rotateMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        rotateMotor.setIdleMode(IdleMode.kBrake);

        // Creates the encoders
        driveEncoder = driveMotor.getEncoder();
        absoluteEncoder = rotateMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // Creates the rotate PID Controller
        rotateMotorController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);
        rotateMotorController.enableContinuousInput(-Math.PI, Math.PI);
        rotateMotorController.setTolerance(0.15); // Wide tolerance to not delay auto

        // Configures the drive encoder
        driveEncoder.setPositionConversionFactor(DRIVE_POS_CONVERSION_FACTOR);   // Converts from revolutions to meters
        driveEncoder.setVelocityConversionFactor(DRIVE_VEL_CONVERSION_FACTOR);   // Converts from revolutions/minute to meters/second
        driveEncoder.setPosition(0.00);

        // Configures the absolute encoder
        absoluteEncoder.setPositionConversionFactor(MODULE_POS_CONVERSION_FACTOR);  // Converts from revolutions to radians
        absoluteEncoder.setVelocityConversionFactor(MODULE_VEL_CONVERSION_FACTOR);  // Converts from revolutions/second to radians/second
        absoluteEncoder.setInverted(true);
    }

    /**
     * Sets the desired state for the module.
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimizes the wheel movements
        /* When you optimize the swerve module state you minimize the wheel rotation.
         * For instance instead of rotating 180 degrees and driving forward you can 
         * just drive in reverse and not rotate.  The optimized state will figure this 
         * out for you.
         */
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d( getAbsPosition() ));

        // Calculates the rotate power
        double currentAngle = getAbsPosition();
        double targetAngle  = MathUtil.angleModulus( optimizedState.angle.getRadians() );
        double rotatePower  = rotateMotorController.calculate(currentAngle, targetAngle);

        // Sets motor powers
        /*  TJM  can we print values to drive and rotate motor.
         *   The rotate motor PID may give us values between -1.0 to 1.0.  
         *   We should clamp at minimum.
         *   these should be -1.0 to 1.0. 
         *  We may need to translate speed to motor power -1.0 to 1.0 using power_speed_ratio
         *  If this is correct we'd need to check for precision mode as well.
         *  This could change our teleop drive results from Saturday 2-3-2024
         */
        driveMotor.set(optimizedState.speedMetersPerSecond);
        rotateMotor.set(rotatePower);
    }

    /**
     * Sets the desired state for the module without optimizations.
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredStateNoOpt(SwerveModuleState desiredState) {
        // Calculates the rotate power
        double currentAngle = getAbsPosition();
        double targetAngle  = MathUtil.angleModulus( desiredState.angle.getRadians() );
        double rotatePower  = rotateMotorController.calculate(currentAngle, targetAngle);

        driveMotor.set(desiredState.speedMetersPerSecond);
        rotateMotor.set(rotatePower);
    }

    /****************************************************************************************** 
    *
    *    GETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Gets the current position of the module.
     * @return The current position of the module.
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            getDrivePosition(),
            new Rotation2d( getAbsPosition() )
        );
    }

    /**
     * Gets the current state of the module.
     * @return The current state of the module.
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            new Rotation2d( getAbsPosition() )
        );
    }

    /**
     * Gets the drive motor position.
     * @return The drive motor's position in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets the drive motor position.
     * @return The drive motor's position in feet
     */
    public double getDrivePositionFeet() {
        return Units.metersToFeet(getDrivePosition());
    }

    /**
     * Gets the drive motor velocity.
     * @return The drive motor's velocity in meters/second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Gets the absolute encoder position.
     * @return The absolute encoder's position in radians
     */
    private double getAbsPosition() {
        /* angle is in radians and using -pi to +pi for modulus */
        return MathUtil.angleModulus( absoluteEncoder.getPosition() );
    }

    /**
     * Checks if the rotate controller is at the setpoint.
     * @return atSetpoint
     */
    public boolean rotateControllerAtSetpoint() {
        return rotateMotorController.atSetpoint();
    }


    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Powers individual drive motor.
     * @param power
     */
    public void setDriveMotorPower(double power) {
        power = MathUtil.clamp(power, -1, 1);     
        driveMotor.set(power);
    }

    /**
     * Powers individual rotate motor.
     * @param power
     */
    public void setRotateMotorPower(double power) {
        power = MathUtil.clamp(power, -1, 1);    
        rotateMotor.set(power);
    }

    /**
     * Inits the motor sliders on SmartDashboard.
     */
    public void initMotorSliders() {
        SmartDashboard.putNumber("Drive Motor Power", 0);
        SmartDashboard.putNumber("Rotate Motor Power", 0);
    }

    /**
     * Displays the encoder values on SmartDashboard.
     */
    public void displayEncoderValues() {
        SmartDashboard.putNumber(driveMotor.getDeviceId()  + "Drive Encoder", getDrivePosition());
        SmartDashboard.putNumber(rotateMotor.getDeviceId() + "Rotation Absolute Encoder", getAbsPosition() );
    }

    /**
     * Powers module through SmartDashboard sliders.
     */
    public void updateMotorPowers() {
        setDriveMotorPower(SmartDashboard.getNumber("Drive Motor Power", 0));
        setRotateMotorPower(SmartDashboard.getNumber("Rotate Motor Power", 0));
    }

    /**
     * Zeros the position of the drive encoder.
     */
    public void zeroDriveEncoder() {
        driveEncoder.setPosition(0.00);
    }

    /**
     * Displays the power and velocity of the drive motor.
     * Conversion Factor 1:
     *  Power 0.25 = 1675rpm
     *  Power 0.50 = 3315rpm
     *  Power 0.75 = 4975rpm
     *  Power 1.00 = 6600rpm
     * 
     * Conversion to Meters Per Second:
     *  Power 0.25 = 1.37m/s - 5.48
     *  Power 0.50 = 2.71m/s - 5.42
     *  Power 0.75 = 4.07m/s - 5.43
     *  Power 1.00 = 5.45m/s - 5.45
     *  AVG - 17.86 ft/s / power
     */
    public void displayPowerAndVelocity() {
        System.out.println(driveMotor.getDeviceId() + " Power " + prevPower + " Velocity " + getDriveVelocity() + " Ratio " + prevPower/getDriveVelocity());
    }
}
// End of the SwerveModule class
