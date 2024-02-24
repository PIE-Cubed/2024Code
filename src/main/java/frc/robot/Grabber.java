package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;

public class Grabber {
    private final int GRABBER_MOTOR1_CAN = 2;
    private final int GRABBER_MOTOR2_CAN = 5;

    private final boolean GRABBER_MOTOR1_IS_INVERTED = false;
    private final boolean GRABBER_MOTOR2_IS_INVERTED = false;

    // TODO tune these power values
    public final double INTAKE_POWER = 0.9;
    public final double OUTTAKE_POWER = 0.75;
    public final double FEED_POWER = 0.75;  
    public final double PROXIMITY_THRESHOLD = 155; 
    public final double IR_THRESHOLD = 150;
    
    private CANSparkMax grabberMotor1;
    private CANSparkMax grabberMotor2;
    private ColorSensorV3 colorSensor;
    private static Grabber instancedGrabber;

    public static synchronized Grabber getInstance() {
        if (instancedGrabber == null){
            instancedGrabber = new Grabber();
        }
        
        return instancedGrabber;
    }
    
    // Constructor
    private Grabber() {
        // Instantiate grabber motors & sensors
        grabberMotor1 = new CANSparkMax(GRABBER_MOTOR1_CAN, MotorType.kBrushed);
        grabberMotor1.setInverted(GRABBER_MOTOR1_IS_INVERTED);

        grabberMotor2 = new CANSparkMax(GRABBER_MOTOR2_CAN, MotorType.kBrushed);
        grabberMotor2.setInverted(GRABBER_MOTOR2_IS_INVERTED);

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        // Set the grabber motor idle modes to break, as it helps stop the note from moving forward
        grabberMotor1.setIdleMode(IdleMode.kBrake);
        grabberMotor2.setIdleMode(IdleMode.kBrake);
    } 

    /**
     * Sets the grabber motor's power
     * 
     * @param power
     */
    public void setMotorPower(double power) {
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
            SmartDashboard.putBoolean("Note detected", noteDetected());

            if(noteDetected()) {
                setMotorPower(0.0);
                return Robot.DONE;
            } 
            else {
                setMotorPower(INTAKE_POWER);
                return Robot.CONT;
            }
        } 
        else if(outtake) {
            setMotorPower(OUTTAKE_POWER);
            return Robot.CONT;
        }
        else {
            setMotorPower(0.0);
            return Robot.DONE;
        }
    }

    /**
     * <p> Checks if the the color sensor senses a note
     * <p> Uses a threshold to determine when to cut power
     * <p> Initial values are ~110 without a note, and ~250 with a note 
     * @return whether a note is found
     */
    public boolean noteDetected() {
        return colorSensor.getProximity() > PROXIMITY_THRESHOLD;
    }

    /**
     * Prints color(r,g,b), IR, and proximity readings from the sensor
     * IR has a range; we've observed values from 110 with no note, to 250 with note
     * Red goes from 0.28 with no note to 0.38 with note
     *
     * IR works best due to its high range/delta, and is more tolerant 
     * to environmental interference from sources such as light
     */
    public void testColorSensor() {
        Color detectedColor = colorSensor.getColor();
        
        double r = 255 * detectedColor.red;
        double g = 255 * detectedColor.green;
        double b = 255 * detectedColor.blue;
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
