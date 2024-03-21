package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTags {
    private NetworkTable aprilTagTable;

    private final double MAX_DISTANCE_FT = 6;   // Maximum distance to speaker until its too unreliable to shoot

    public AprilTags() {
        aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * <p> Calculate the angle the arm needs to be at to get into the speaker
     * <p> Currently only uses apriltag 7
     * @return The elevation of the arm to shoot into the speaker
     *         <p> -1 if it fails to get the AprilTag
     */
    public double calculateArmAngleToShoot() {
        // To be implemented...

        return -1;
    }

    /**
     * <p>Gets the distance to the target AprilTag in meters
     * <p>The id param is only used to ensure the correct AprilTag is found
     * <p>Returns -1 if it fails to get the AprilTag
     * <p>Using 'LimelightHelpers' as NetworkTables crashes when getting pose
     * @param pipeline The limelight pipeline to searchfor the AprilTag
     * @param id The ID of the target AprilTag
     * @return Distance to AprilTag in meters
     */
    public double getDistanceToAprilTagMeters(int pipeline, int id) {
        LimelightHelpers.setPipelineIndex("limelight", pipeline);   // Set the pipeline
        
        if(aprilTagTable.getEntry("tv").getDouble(0.0) == 1 &&
           aprilTagTable.getEntry("tid").getDouble(0.0) == id){
            /* Pose3d:
             *  x: The horizontal offset
             *  y: The vertical offset
             *  z: The forward offset
             */
            Pose3d targetpose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
            double x = targetpose.getX();
            double z = targetpose.getZ();

            double distance = Math.sqrt((x*x) + (z*z)); // Calculate distance with pythagorean's formula

            return distance;
        }
        return -1;
    }

    /**
     * @param pipeline The pipeline for the AprilTag
     * @param id The ID of the AprilTag, used for redundancy
     * @return If the robot is too far away from the speaker
     */
    public boolean outOfRange(int pipeline, int id) {
        return Units.metersToFeet(getDistanceToAprilTagMeters(pipeline, id)) > MAX_DISTANCE_FT;
    }

    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/  
    public void testAprilTagID() {
        double tid = aprilTagTable.getEntry("tid").getDouble(0.0);
        double tv = aprilTagTable.getEntry("tv").getDouble(0.0);

        SmartDashboard.putNumber("LimelightID", tid);
        SmartDashboard.putNumber("LimelightValidTag", tv);
    }

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
        double tx = aprilTagTable.getEntry("tx").getDouble(0.0);    // Target x
        double ty = aprilTagTable.getEntry("ty").getDouble(0.0);    // Target y
        double ta = aprilTagTable.getEntry("ta").getDouble(0.0);    // Target area

        // Post to smart dashboard
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
    }

    public void testAprilTagPipeline(int pipeline) {
        aprilTagTable.getEntry("pipeline").setNumber(pipeline); // Set the pipeline
        
        double pipelineNT = aprilTagTable.getEntry("pipeline").getDouble(0);    // Check if the change is reflected in NetworkTables
        SmartDashboard.putNumber("LimelightPipeline", pipelineNT);  // Display current pipeline from NetworkTables
    }
}
