package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  public static class FieldConstants {
    public static final double SPK_HEIGHT = 2.03;
    public static final Translation2d SPK_POS = new Translation2d(16.54, 5.555);
    public static final Translation2d[] BLOCKED_ASTAR_NODES = null;

    public static final Translation2d[] NOTE_POS = {
      new Translation2d(2.9, 4.105), new Translation2d(2.9, 5.555), new Translation2d(2.9, 7.005),
      new Translation2d(8.27, .745), new Translation2d(8.27, 2.425), new Translation2d(8.27, 4.105), new Translation2d(8.27, 5.785), new Translation2d(8.27, 7.465),
      new Translation2d(13.64, 4.105), new Translation2d(13.64, 5.555), new Translation2d(13.64, 7.005)
    };
  }

  public static class ModuleConstants {
      // Wheel specifications
      public static final double WHEEL_DIAMETER = 0.1; //In meters.

      // Motor ratios
      public static final double PWR_RATIO = 1.0 / 6.12; // Power motor ratio.
      public static final double STR_RATIO = 1.0 / 12.8; // Turning motor gear ratio.

      // Encoder conversions
      public static final double ENC_ROT_2_M = PWR_RATIO * Math.PI * WHEEL_DIAMETER; 
      public static final double ENC_RPM_2_M_S = 5800 / 60.0 * ENC_ROT_2_M; // In meters per second.
      public static final double TURNING_ROT_2_RAD = STR_RATIO * 2 * Math.PI; 
      public static final double TURNING_RPM_2_RAD_S = 5676 / 60.0 * TURNING_ROT_2_RAD;

      // PID constants
      public static final double PID_P = 0.217; // Proportional gain.
      public static final double PID_I = 0; // Integral gain.
      public static final double PID_D = 0.0; // Derirvative gain.

      // Encoder offsets
      public static final double[] ENCODER_OFFSETS = {0, 0, 0, 0}; // {FL, FR, BL, BR} offsets.

  
  }

  public static class ChassisConstants {
    public static final double TRACKWIDTH = .537;// Distance between right and left wheels.
    public static final double WHEELBASE = .380; // Distance between front and back wheels.
    public static final double ROBOT_WEIGHT = 45;
        
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(TRACKWIDTH / 2, WHEELBASE / 2),
      new Translation2d(TRACKWIDTH / 2, -WHEELBASE / 2),
      new Translation2d(-TRACKWIDTH / 2, WHEELBASE / 2),
      new Translation2d(-TRACKWIDTH / 2, -WHEELBASE / 2));

    // Speed calculations
    public static final double MAX_SPD = 4; // in m/s.
    public static final double MAX_ANG_SPD = 2 * Math.PI; // in radians per second.
    
    // Maximum accelerations
    public static final double MAX_ACCEL = 3;
    public static final double MAX_ANG_ACCEL = 3;
  }

  public static class AutonomousConstants {
    public static final Pose2d initialPose = new Pose2d();

    public static final double P = 1.5;
    public static final double I = 0;
    public static final double D = 0;

    public static final double P_Z = 3;
    public static final double I_Z = 0;
    public static final double D_Z = 0;

    public static final double MAX_SPD = ChassisConstants.MAX_SPD; // In meters per second.
    public static final double MAX_ACCEL = ChassisConstants.MAX_ACCEL; // In meters per second^2.
    public static final double MAX_ANG_SPD = ChassisConstants.MAX_ANG_SPD; // In radians per second.
    public static final double MAX_ANG_ACCEL = ChassisConstants.MAX_ANG_ACCEL; // In radians per second^2.

    public static final TrapezoidProfile.Constraints Z_CONTROLER = 
                new TrapezoidProfile.Constraints(
                        MAX_ANG_SPD,
                        MAX_ANG_ACCEL
    );

    public static final double MAX_DIST_SPK = 5;
  }

  public static class OperatorConstants {
    //--------Driver constants--------

      // Joystick settings
    public static final int DRIVER_PORT = 0;

      // Sticks
    public static final int DRIVER_X = 1;
    public static final int DRIVER_Y = 0;
    public static final int DRIVER_Z = 4;

      // Buttons
    public static final int ZERO_HDG = 1;
    public static final int ROBOT_ORIENTED = 5;

    //--------Co-Driver constants--------

      // Joystick settings
    public static final int CODRIVER_PORT = 1;

      // Sticks
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
      // Buttons
    public static final int RELOAD = 5;
    public static final int CALIBRATE = 6;
    public static final int CALIBRATE1 = 7;

    // General constants
    public static final double JOYSTICK_DEADZONE = .1;
  }

  public static class HardwareMap {
    //--------Rev Robotics--------

    // SPARKS
    public static final int FL_STR = 1;
    public static final int FR_STR = 2;
    public static final int BL_STR = 3;
    public static final int BR_STR = 4;

    public static final int ELEVATOR = 5;
    public static final int ELEVATOR1 = 6;

    public static final int ROLLERS = 7;
    public static final int ROLLERS1 = 8;
    public static final int CLAW = 9;
    public static final int ANGLE = 10;

    public static final int ARM = 11;

    //--------CTR Electronics--------

    // Krakens 
    public static final int FL_PWR = 1;
    public static final int FR_PWR = 2;
    public static final int BL_PWR = 3;
    public static final int BR_PWR = 4;

    // Swerve encoders
    public static final int FL_ENC = 5;
    public static final int FR_ENC = 6;
    public static final int BL_ENC = 7;
    public static final int BR_ENC = 8;

    // Gyro
    public static final int PIGEON = 9;

    //------------roboRIO--------------

    // DIO
    public static final int ELEVATOR_ENC = 0;
    public static final int BOTTOM_LIMIT = 1;
    public static final int TOP_LIMIT = 2;

    public static final int CLAW_ENC = 3;
    public static final int ANGLE_ENC = 4;

    public static final int ARM_ENC = 5;
  }

  public static class ElevatorConstants {
    public static final double MAX_HEIGHT = 1.6675;

    public static final double P = 0.217;
    public static final double I = 0.0;
    public static final double D = 0.0;
    
  }

  public static class ClawConstants {
    public static final double CLAW_P = 0.217;
    public static final double CLAW_I = 0.0;
    public static final double CLAW_D = 0.0;

    public static final double ANGLE_P = 0.217;
    public static final double ANGLE_I = 0.0;
    public static final double ANGLE_D = 0.0;

    public static final double MAX_ANGLE = 88.4;
    public static final double MIN_ANGLE = -88.4;
  }

  public static class ArmConstants {
    public static final double P = 0.217;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final double MAX_ANGLE = 116;
    public static final double MIN_ANGLE = -104;
  }

  public static class VisionConstants {
    public static final Transform3d[] ROBOT_TO_CAM = {
      new Transform3d(new Translation3d(), new Rotation3d()), // Camera 1.
      new Transform3d(new Translation3d(), new Rotation3d()) // Camera 2.
    };
    
    public static final double CAM_ANGLE[] = {
      Math.toRadians(62.5),
      Math.toRadians(62.5)
    }; // The view angle of the camera.
  }
}
