package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LED {
    private Spark ledController;
    private int ledID = 0;

    /* 
     * Use table at:
     *  https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
     *  Pages 14-17
    */
    private double TEAM_COLORS = 0.41;      // Color gradient between color 1 and 2 (set on controller physically)
    private double PARTY_COLORS = -0.97;    // Rainbow party
    private double PRECISION_MODE = 0.93;   // White
    private double GREEN = 0.77;            // Green
    private double ORANGE = 0.65;           // Orange
    private double YELLOW = 0.69;           // Yellow
    private double RED = 0.61;              // Red
    
    private double oldColor;
    private double setColor = TEAM_COLORS;

    public LED() {
        ledController = new Spark(ledID);
        ledController.set(TEAM_COLORS);
    }

    public void updateLED() {
        if(oldColor != setColor) {
            ledController.set(setColor);
        }  
        oldColor = setColor;
    }

    /**
     * Sets the LEDs to blue and gold
     */
    public void robolionsColor() {
        setColor = TEAM_COLORS;
    }

    /**
     * Sets the LEDs to green
     */
    public void capturedNoteColor() {
        setColor = GREEN;
    }

    /**
     * Sets the LEDs to orange
     */
    public void gettingNoteColor() {
        setColor = ORANGE;
    }

    /**
     * Sets the LEDs to red
     */
    public void apriltagOutOfRangeColor() {
        setColor = RED;
    }

    /**
     * Sets the LEDs to orange
     */
    public void apriltagInRangeMovingColor() {
        setColor = ORANGE;
    }

    /**
     * Sets the LEDs to green
     */
    public void apriltagReadyToShootColor() {
        setColor = GREEN;
    }

    /**
     * Sets the LEDs to rainbow
     */
    public void partyColor() {
        setColor = PARTY_COLORS;
    }

}
