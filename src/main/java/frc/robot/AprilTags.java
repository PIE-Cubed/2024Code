package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTags {
    private NetworkTable aprilTagTable;

    private boolean isRed = false;

    private final double MAX_DISTANCE_FT = 6;   // Maximum distance to speaker until its too unreliable to shoot
    private final double CUBED_CONST = 0.00000325;
    private final double SQUARED_CONST = 0.00317;
    private final double SINGLE_CONST = 1.09;
    private final double OFFSET = 228;

    public AprilTags(boolean isRed) {
        //System.out.println("[INFO] >> Configuring limelight...");
        // Turn off the limelight LEDs so they don't blind people
        LimelightHelpers.setLEDMode_ForceOff("limelight");

        //aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight");
        //System.out.println("[INFO] >> Getting alliance...");
        this.isRed = isRed;
    }

    /**
     * <p> Calculate the angle the arm needs to be at to get into the speaker
     * <p> Currently only uses apriltag 7
     * @return The elevation of the arm to shoot into the speaker
     *         <p> -1 if it fails to get the AprilTag
     */
    public double calculateArmAngleToShoot() {
        //double distance = getDistanceToSpeakerFeet() * 30.48;
        //double angle = (CUBED_CONST * Math.pow(distance, 3)) - (SQUARED_CONST * Math.pow(distance, 2)) + (SINGLE_CONST * distance) + OFFSET;
        double distance = getDistanceToSpeakerFeet();
        double angle = (0.0331 * Math.pow(distance, 3)) - (1.2643 * Math.pow(distance, 2)) + (17.396 * distance) + 277.39;

        if(angle >= 360) {
            angle -= 360;
        }

        // If the distance is >= 8 feet, subtract 1.5 degrees
        // for every foot past the 8 feet
        //if (distance >= 243.84) { // 8 feet
        //    angle -= Math.abs(1.51 * ((distance / 30.48) - 8));
        //    angle = Math.abs(angle);
        //}
        if(angle <= 330 && angle >= 5) {
            System.out.println("Angle(" + angle + ") out of range (5-330deg)! Using 339");
            angle = 339;
        }
        //System.out.println(distance);

        return angle;
    }

    /**
     * <p>Gets the distance to the target AprilTag in meters
     * <p>The id param is only used to ensure the correct AprilTag is found
     * <p>Returns -1 if it fails to get the AprilTag
     * <p>Reports a ~3-4in greater distance
     * <p>Must call 'setSpeakerPipeline' first
     * @return Distance to AprilTag in meters
     */
    public double getDistanceToSpeakerFeet() {
        if(validApriltagInView()) {
            /* Pose3d:
             *  x: The horizontal offset
             *  y: The vertical offset
             *  z: The forward offset
             */
            Pose3d targetpose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
            double x = targetpose.getX();
            double z = targetpose.getZ();

            double distance = Math.sqrt((x*x) + (z*z)); // Calculate distance with pythagorean's formula

            return Units.metersToFeet(distance);
        }
        return -1;
    }

    /**
     * Checks if the robot is too far to shoot
     * @return If the robot is too far away from the speaker
     */
    public boolean outOfRange() {
        return Units.metersToFeet(getDistanceToSpeakerFeet()) > MAX_DISTANCE_FT;
    }

    /**
     * Gets the offset of the AprilTag to the center of the limelight's view
     * @return The degree offset
     */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX("limelight");
    }

    public boolean validApriltagInView() {
        return LimelightHelpers.getTV("limelight");
    }

    /**
     * <p> Sets the limelight pipeline for the speaker
     * <p> Pipeline 0 for Apriltag 4(Red)
     * <p> Pipeline 1 for Apriltag 7(Blue)
     */
    public void setSpeakerPipeline() {
        if(isRed){
            LimelightHelpers.setPipelineIndex("limelight", 0);  // AprilTag 4
        }
        else {
            LimelightHelpers.setPipelineIndex("limelight", 1);  // AprilTag 7
        }
    }

    /**
     * <p> Sets the limelight pipeline for the speaker
     * <p> Use this to manually set the speaker pipeline
     * <p> Pipeline 0 for Apriltag 4(Red)
     * <p> Pipeline 1 for Apriltag 7(Blue)
     * @param isRed If we're on the red alliance
     */
    public void setSpeakerPipeline(boolean isRed) {
        if(isRed){
            LimelightHelpers.setPipelineIndex("limelight", 0);  // AprilTag 4
        }
        else {
            LimelightHelpers.setPipelineIndex("limelight", 1);  // AprilTag 7
        }
    }

    /**
     * Sets the limelight pipeline
     * @param pipeline The pipeline ID
     */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex("limelight", pipeline);
    }

    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/  
    public void testAprilTagXY() {
        /*
         * tx:
         *  Horizontal offset from crosshair to target
         *  Positive is to the right relative to the camera
         *  LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees
         * ty:
         *  Vertical offset from crosshair to target
         *  Positive is downwards relative to the camera
         *  LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees
         * ta:
         *  Target area percent, 0% - 100% of image
         */
        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");
        double ta = LimelightHelpers.getTA("limelight");

        // Post to smart dashboard
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
    }
    public int getAprilTagID() {
        return (int)LimelightHelpers.getFiducialID("limelight");
    }

    public void setLED(boolean on) {
        if(on) {
            LimelightHelpers.setLEDMode_ForceOn("limelight");
        }
        else {
            LimelightHelpers.setLEDMode_ForceOff("limelight");
        }
    }
}
