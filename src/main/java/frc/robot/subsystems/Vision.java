package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.VisionConstants;

public class Vision extends SubsystemBase{
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    public int getAprilDetections(int cameraId){
        double count = limelightTable.getEntry("apriltagCount").getDouble(cameraId);

        return (int) count;
    }
    public double getAprilDistance(int cameraId){
        double ty = limelightTable.getEntry("ty").getDouble(cameraId);

        double mountAngleRadians = Math.toRadians(VisionConstants.CAM_ANGLE[cameraId]);
      double tyRadians = Math.toRadians(ty);
      

      double distance = (VisionConstants.APRILTAG_WIDTH - VisionConstants.ROBOT_TO_CAM[0].getY()) / Math.tan(mountAngleRadians + tyRadians);
      return distance;
    }

    public double[] getRobotPoseFromFirstAprilTag(int cameraId){
        if(getAprilDetections(cameraId) > 0) {
            double[] botpose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
            return botpose;
        }
        return new double[0];
    }
  }
