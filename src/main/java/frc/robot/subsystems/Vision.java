package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Constants.VisionConstants;

public class Vision extends SubsystemBase{
    public int getAprilDetections(String cameraName){
        //System.out.println(LimelightHelpers.getTargetCount(cameraName));
        return LimelightHelpers.getTargetCount(cameraName);
    }

    public LimelightHelpers.PoseEstimate getRobotPoseFromAprilTags(double robotYaw, double robotPitch, double robotRoll, boolean redAlliance) {
        LimelightHelpers.setCropWindow("limelight-three", -0.5, 0.5, -0.5, 0.5);

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-three", 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        );

        LimelightHelpers.setFiducial3DOffset(
            "limelight-three", 
            0, 
            0, 
            0
        );

        LimelightHelpers.SetRobotOrientation("limelight-three", robotYaw, 0.0, robotPitch, 0.0, robotRoll, 0.0);
        if (redAlliance) {
            return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-three");
        } else {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-three");
        }
        
        
    }
  }
