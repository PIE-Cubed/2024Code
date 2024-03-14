package frc;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Climber {
    private final int LEFT_MOTOR_CAN = -1;
    private final int RIGHT_MOTOR_CAN = -1;
    private final int MOTOR_CURRENT_LIMIT = 70;
    
    private final double CLIMB_POWER = 0.5;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    
    private RelativeEncoder leftMotorEncoder;
    private RelativeEncoder rightMotorEncoder;
    
    private PIDController leftMotorPidController;
    private PIDController rightMotorPidController;

    public Climber() {
        // Motors
        leftMotor = new CANSparkMax(LEFT_MOTOR_CAN, MotorType.kBrushless);
        leftMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        leftMotor.setIdleMode(IdleMode.kBrake);
        
        rightMotor = new CANSparkMax(RIGHT_MOTOR_CAN, MotorType.kBrushless);
        rightMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        rightMotor.setIdleMode(IdleMode.kBrake);

        // Encoders
        leftMotorEncoder = leftMotor.getEncoder();
        rightMotorEncoder = rightMotor.getEncoder();
    }

    
    /**
     * <p> Climb by setting motor power
     * <p> DOES NOT STOP ON ITS OWN
     */
    public void climbTest() {
        leftMotor.set(CLIMB_POWER);
        rightMotor.set(CLIMB_POWER);
    }
}
