// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.CANSparkFlex;

/** 
 * Controls the shooting mechanism
 */

 public class Shooter {

    private final int SHOOTER_MOTOR_1_CAN = 20;
    private final int SHOOTER_MOTOR_2_CAN = 21;
    private final int MOTOR_CURRENT_LIMIT = 70;
    private CANSparkFlex shooterMotor1;
    private CANSparkFlex shooterMotor2;

    public Shooter(){
        shooterMotor1 = new CANSparkFlex(SHOOTER_MOTOR_1_CAN, MotorType.kBrushless);
        shooterMotor1.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        shooterMotor1.setIdleMode(IdleMode.kBrake);
        
        shooterMotor2 = new CANSparkFlex(SHOOTER_MOTOR_2_CAN, MotorType.kBrushless);
        shooterMotor2.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        shooterMotor2.setIdleMode(IdleMode.kBrake);
    }

    public void startShooting(double shootPower) {
        shooterMotor1.set(MathUtil.clamp(shootPower, -1.0, 1.0));
        shooterMotor2.set(MathUtil.clamp(shootPower, -1.0, 1.0));
    }

    public void stopShooting() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    public void spinup() {
        shooterMotor1.set(0.75);
        shooterMotor2.set(0.75);
    }

}
