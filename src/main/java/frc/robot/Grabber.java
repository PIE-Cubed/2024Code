package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class Grabber {
    private final int GRABBER_MOTOR_CAN = 0;
    private final boolean GRABBER_MOTOR_IS_INVERTED = false;

    public final double INTAKE_POWER = 0.75;
    public final double OUTTAKE_POWER = 0.75;

    private CANSparkMax grabberMotor;
    
    public Grabber() {
        grabberMotor = new CANSparkMax(GRABBER_MOTOR_CAN, MotorType.kBrushed);
        grabberMotor.setInverted(GRABBER_MOTOR_IS_INVERTED);
    } 

    /**
     * Sets the grabber motor's power
     * 
     * @param power
     */
    public void setPower(double power) {
        grabberMotor.set(MathUtil.clamp(power, -1, 1));
    }

    public void intakeOutake(boolean intake, boolean outtake) {
        if(intake) {
            setPower(INTAKE_POWER);
        } else if(outtake) {
            setPower(OUTTAKE_POWER);
        }
        else {
            setPower(0.0);
        }
    }
}
