package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    public static final double MAX_VELOCITY = 5; // <-- in m/s.
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

    public static final double POS_TOLERANCE = 0.05; // <-- Position tolerance in meters.
    public static final double ANG_TOLERANCE = Math.toRadians(5); // <-- Angular tolerance in radians.

    public static final double NORMAL_STD = 0.003;

    public static final Matrix<N3, N1> NORMAL_CONFIDENCE_STD;

    static {
      NORMAL_CONFIDENCE_STD = new Matrix<>(N3.instance, N1.instance);
      NORMAL_CONFIDENCE_STD.set(0, 0, NORMAL_STD); // X
      NORMAL_CONFIDENCE_STD.set(1, 0, NORMAL_STD); // Y
      NORMAL_CONFIDENCE_STD.set(2, 0, Math.toRadians(3)); // Z
    }
    public static final double LOW_STD = 0.05; // Define a standard deviation value.

    public static final Matrix<N3, N1> LOW_CONFIDENCE_STD;

    static {
        LOW_CONFIDENCE_STD = new Matrix<>(N3.instance, N1.instance);
        LOW_CONFIDENCE_STD.set(0, 0, LOW_STD); // X
        LOW_CONFIDENCE_STD.set(1, 0, LOW_STD); // Y
        LOW_CONFIDENCE_STD.set(2, 0, Math.toRadians(5)); // Z
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
