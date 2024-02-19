package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

public class Grabber {
    private final int GRABBER_MOTOR1_CAN = 2;
    private final int GRABBER_MOTOR2_CAN = 5;

    private final boolean GRABBER_MOTOR1_IS_INVERTED = false;
    private final boolean GRABBER_MOTOR2_IS_INVERTED = true;

    // TODO tune these power values
    public final double INTAKE_POWER = 0.75;
    public final double OUTTAKE_POWER = 0.75;
    public final double FEED_POWER = 0.75;
    
    public final double IR_THRESHOLD = 200;
    
    private CANSparkMax grabberMotor1;
    private CANSparkMax grabberMotor2;
    private ColorSensorV3 colorSensor;
    
    public Grabber() {
        grabberMotor1 = new CANSparkMax(GRABBER_MOTOR1_CAN, MotorType.kBrushed);
        grabberMotor1.setInverted(GRABBER_MOTOR1_IS_INVERTED);

        grabberMotor2 = new CANSparkMax(GRABBER_MOTOR2_CAN, MotorType.kBrushed);
        grabberMotor2.setInverted(GRABBER_MOTOR2_IS_INVERTED);

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    } 

    /**
     * Sets the grabber motor's power
     * 
     * @param power
     */
    public void setPower(double power) {
        grabberMotor1.set(MathUtil.clamp(power, -1, 1));
        grabberMotor2.set(MathUtil.clamp(power, -1, 1));
    }

    /**
     * <p> Sets the motor power to feed power
     * <p> Cuts power when a note is detected
     * 
     * @return Robot status, CONT if there is no note, DONE if there is
     * 
     */
    // TODO test color sensor's ability to stop intake correctly
    public int intakeOutake(boolean intake, boolean outtake) {
        if(intake) {
            if(noteDetected()) {
                return Robot.DONE;
            } else {
                setPower(INTAKE_POWER);
                return Robot.CONT;
            }
        } else if(outtake) {
            setPower(OUTTAKE_POWER);
            return Robot.CONT;
        }
        else {
            setPower(0.0);
            return Robot.DONE;
        }
    }

    /**
     * <p> Checks if the the color sensor senses a note
     * <p> Uses a threshold to determine when to cut power
     * <p> Initial values are ~110, and ~250 with a note
     * 
     * @return whether a note is found
     */
    public boolean noteDetected() {
        return colorSensor.getIR() > IR_THRESHOLD;
    }

    /**
     * Prints color(r,g,b), ir, and proximity readings from the sensor
     * IR has a range, from 110 with no note, to 250 with note
     * Red goes from 0.28 with no note to 0.38 with note
     * IR works best due to its high range/delta
     */
    public void testColorSensor() {
        Color detectedColor = colorSensor.getColor();
        
        double r = detectedColor.red;
        double g = detectedColor.green;
        double b = detectedColor.blue;
        double ir = colorSensor.getIR();
        double proximity = colorSensor.getProximity();
        
        System.out.println( "COLOR SENSOR" +
            " | R: "  + r  +
            " | G: "  + g  +
            " | B: "  + b  +
            " | IR: " + ir +
            " | Proximity: " + proximity
        );
    }

}
