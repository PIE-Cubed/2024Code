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

    private final int SHOOTER_MOTOR_1_CAN = 21;
    private final int SHOOTER_MOTOR_2_CAN = 22;
    private final int MOTOR_CURRENT_LIMIT = 70;
    private final double DUMP_SHOT_POWER  = 0.25;   // TODO tune power value
    private CANSparkFlex shooterMotor1;
    private CANSparkFlex shooterMotor2;
    private Grabber instancedGrabber;

    public Shooter(){
        // Instantiate shooter motors
        shooterMotor1 = new CANSparkFlex(SHOOTER_MOTOR_1_CAN, MotorType.kBrushless);
        shooterMotor1.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        shooterMotor1.setIdleMode(IdleMode.kCoast);
        
        shooterMotor2 = new CANSparkFlex(SHOOTER_MOTOR_2_CAN, MotorType.kBrushless);
        shooterMotor2.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        shooterMotor2.setIdleMode(IdleMode.kCoast);

        // Get the same grabber instance
        instancedGrabber = Grabber.getInstance();

        // Spin the motors
        //spinup();
    }

    /**
     * Shoots the note at low power
     */
    public void doDumpShot() {
        startShooting(DUMP_SHOT_POWER);
    }

    /**
     * Set power to spin motors, might not be needed if there's only two power values to use 
     * @param shootPower The power to shoot at
     */
    public void startShooting(double shootPower) {
        shooterMotor1.set(MathUtil.clamp(shootPower, -1.0, 1.0));
        shooterMotor2.set(MathUtil.clamp(shootPower, -1.0, 1.0));
        instancedGrabber.intakeOutake(true, false);
    }

    public void stopShooting() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
        instancedGrabber.intakeOutake(false, false);
    }

    /**
     * Spinup motor, to be ready at all times
     */
    public void spinup() {
        shooterMotor1.set(1);
        shooterMotor2.set(1);
    }

    /**********************************************************************
     * 
     *    TEST FUNCTIONS
     * 
     **********************************************************************/
    public void testSpin() {
        shooterMotor2.set(0.05);
    }

}
