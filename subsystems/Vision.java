package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class Vision extends SubsystemBase {

    public int getAprilDetections(String cameraName) {
        // Returns the number of AprilTag detections from the specified camera.
        // System.out.println(LimelightHelpers.getTargetCount(cameraName)); // <-- Debugging line (commented out).
        return LimelightHelpers.getTargetCount(cameraName); // <-- Retrieves the target count from LimelightHelpers.
    }

    public LimelightHelpers.PoseEstimate getRobotPoseFromAprilTags(double robotYaw, double robotPitch, double robotRoll, boolean redAlliance) {
        // Sets the crop window for the camera to limit the field of view.
        LimelightHelpers.setCropWindow("limelight-three", -0.5, 0.5, -0.5, 0.5); // <-- Configures the crop window.

        // Sets the camera pose relative to the robot's space.
        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-three", 
            0, // <-- X position of the camera relative to the robot.
            0, // <-- Y position of the camera relative to the robot.
            0, // <-- Z position of the camera relative to the robot.
            0, // <-- Yaw angle of the camera.
            0, // <-- Pitch angle of the camera.
            0  // <-- Roll angle of the camera.
        );

        // Sets the 3D offset for fiducial markers detected by the camera.
        LimelightHelpers.setFiducial3DOffset(
            "limelight-three", 
            0, // <-- X offset for fiducial markers.
            0, // <-- Y offset for fiducial markers.
            0  // <-- Z offset for fiducial markers.
        );

        // Sets the robot's orientation based on yaw, pitch, and roll values.
        LimelightHelpers.SetRobotOrientation("limelight-three", robotYaw, 0.0, robotPitch, 0.0, robotRoll, 0.0); // <-- Updates robot orientation.

        if (redAlliance) {
            // Retrieves the robot's pose estimate for the red alliance using AprilTags.
            return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-three"); // <-- Pose estimate for red alliance.
        } else {
            // Retrieves the robot's pose estimate for the blue alliance using AprilTags.
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-three"); // <-- Pose estimate for blue alliance.
        }
    }
}
