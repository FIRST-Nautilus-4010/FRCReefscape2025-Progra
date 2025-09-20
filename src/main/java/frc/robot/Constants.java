package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  public static class ChassisConstants {
    public static final double TRACKWIDTH = .537;// <-- Distance between right and left wheels.
    public static final double WHEELBASE = .380; // <-- Distance between front and back wheels.
        
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(TRACKWIDTH / 2, WHEELBASE / 2),
      new Translation2d(TRACKWIDTH / 2, -WHEELBASE / 2),
      new Translation2d(-TRACKWIDTH / 2, WHEELBASE / 2),
      new Translation2d(-TRACKWIDTH / 2, -WHEELBASE / 2));

    // Speed calculations
    public static final double MAX_VELOCITY = 5.4; // <-- in m/s.
    public static final double MAX_ANG_SPD = 4.79 * Math.PI; // <-- in radians per second.
    
    // Maximum accelerations
    public static final double MAX_ACCEL = 479; // <-- Maximum linear acceleration in meters per second^2.
    public static final double MAX_ANG_ACCEL = 479; // <-- Maximum angular acceleration in radians per second^2.
  }

  public static class AutonomousConstants {
    public static final Pose2d initialPose = new Pose2d(); // <-- Initial pose of the robot.

    public static final double P_X = 1.5; // <-- Proportional gain for autonomous control.
    public static final double I_X = 0; // <-- Integral gain for autonomous control.
    public static final double D_X = 0; // <-- Derivative gain for autonomous control.

    public static final double P_Y = 1.5; // <-- Proportional gain for autonomous control.
    public static final double I_Y = 0; // <-- Integral gain for autonomous control.
    public static final double D_Y = 0; // <-- Derivative gain for autonomous control.

    public static final double P_Z = 3; // <-- Proportional gain for Z-axis control.
    public static final double I_Z = 0; // <-- Integral gain for Z-axis control.
    public static final double D_Z = 0; // <-- Derivative gain for Z-axis control.

    public static final double MAX_SPD = ChassisConstants.MAX_VELOCITY; // <-- Maximum speed in meters per second.
    public static final double MAX_ACCEL = ChassisConstants.MAX_ACCEL; // <-- Maximum acceleration in meters per second^2.
    public static final double MAX_ANG_SPD = ChassisConstants.MAX_ANG_SPD; // <-- Maximum angular speed in radians per second.
    public static final double MAX_ANG_ACCEL = ChassisConstants.MAX_ANG_ACCEL; // <-- Maximum angular acceleration in radians per second^2.

    public static final TrapezoidProfile.Constraints Z_CONTROLER = 
                new TrapezoidProfile.Constraints(
                        MAX_ANG_SPD, // <-- Maximum angular speed for Z-axis control.
                        MAX_ANG_ACCEL // <-- Maximum angular acceleration for Z-axis control.
    );


    public enum FieldTarget {
      REEF(new Pose3d[] {
          new Pose3d(), // <-- Poses for the reef targets.
      }),
      SOURCE(new Pose3d[] {
          new Pose3d() // <-- Poses for the source targets.
      }),
      ALGAE(new Pose3d[] {
          new Pose3d()
      });
  
      private final Pose3d[] poses;
  
      FieldTarget(Pose3d[] poses) {
          this.poses = poses;
      }
  
      public Pose3d[] getPoses() {
          return poses;
      }

      public Pose3d getPose(int index) {
        return poses[index];
      }
      
      public int getPoseCount() {
          return poses.length;
      }

    }
  
  }

  public static class VisionConstants {
    public static final int CAM_NUM = 1; // <-- Number of cameras.
    public static final Transform3d[] ROBOT_TO_CAM = {
      new Transform3d(new Translation3d(), new Rotation3d()), // <-- Camera 1.
      new Transform3d(new Translation3d(), new Rotation3d()) // <-- Camera 2.
    };
    
    public static final double CAM_ANGLE[] = {
      Math.toRadians(62.5),
      Math.toRadians(62.5)
    }; // The view angle of the camera.

    public static final double APRILTAG_WIDTH = 0.1; // <-- The width of the apriltag.
  }
}
