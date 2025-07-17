package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  public static class ModuleConstants {
      // Wheel specifications
      public static final double WHEEL_DIAMETER = 0.1; // <-- In meters.

      // Motor ratios
      public static final double PWR_RATIO = 1.0 / 6.12; // <-- Power motor ratio.
      public static final double STR_RATIO = 1.0 / 12.8; // <-- Turning motor gear ratio.

      // Encoder conversions
      public static final double ENC_ROT_2_M = PWR_RATIO * Math.PI * WHEEL_DIAMETER; 
      public static final double ENC_RPM_2_M_S = 5800 / 60.0 * ENC_ROT_2_M; // <-- In meters per second.
      public static final double TURNING_ROT_2_RAD = STR_RATIO * 2 * Math.PI; 
      public static final double TURNING_RPM_2_RAD_S = 5676 / 60.0 * TURNING_ROT_2_RAD;

      // PID constants
      public static final double PID_P = 0.097; // <-- Proportional gain.
      public static final double PID_I = 0.000000004010; // <-- Integral gain.
      public static final double PID_D = 0.000267; // <-- Derirvative gain.

      // Encoder offsets
      public static final double[] ENCODER_OFFSETS = {0, 0, 0, 0}; // <-- {FL, FR, BL, BR} offsets.

  
  }

  public static class ChassisConstants {
    public static final double TRACKWIDTH = .537;// <-- Distance between right and left wheels.
    public static final double WHEELBASE = .380; // <-- Distance between front and back wheels.
    public static final double ROBOT_WEIGHT = 45;
        
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(TRACKWIDTH / 2, WHEELBASE / 2),
      new Translation2d(TRACKWIDTH / 2, -WHEELBASE / 2),
      new Translation2d(-TRACKWIDTH / 2, WHEELBASE / 2),
      new Translation2d(-TRACKWIDTH / 2, -WHEELBASE / 2));

    // Speed calculations
    public static final double MAX_SPD = 3; // <-- in m/s.
    public static final double MAX_ANG_SPD = 2 * Math.PI; // <-- in radians per second.
    
    // Maximum accelerations
    public static final double MAX_ACCEL = 3; // <-- Maximum linear acceleration in meters per second^2.
    public static final double MAX_ANG_ACCEL = 1.5; // <-- Maximum angular acceleration in radians per second^2.
  }

  public static class AutonomousConstants {
    public static final Pose2d initialPose = new Pose2d(); // <-- Initial pose of the robot.

    public static final double P = 1.5; // <-- Proportional gain for autonomous control.
    public static final double I = 0; // <-- Integral gain for autonomous control.
    public static final double D = 0; // <-- Derivative gain for autonomous control.

    public static final double P_Z = 3; // <-- Proportional gain for Z-axis control.
    public static final double I_Z = 0; // <-- Integral gain for Z-axis control.
    public static final double D_Z = 0; // <-- Derivative gain for Z-axis control.

    public static final double MAX_SPD = ChassisConstants.MAX_SPD; // <-- Maximum speed in meters per second.
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
    public static final int CODRIVER_X = 1;
    public static final int CODRIVER_Y = 0;
    public static final int CODRIVER_Z = 4;
    public static final int CODRIVER_A = 3;
    
      // Buttons
    public static final int CALIBRATE = 6;
    public static final int CALIBRATE1 = 7;
    public static final int AUTO = 8;
    public static final int ROLLERS = 5;

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
    public static final int ARM1 = 12;

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
    public static final double MAX_HEIGHT = 8840000;

    public static final double P = 0.217;
    public static final double I = 0.0;
    public static final double D = 0.0;

    // Creates a new set of trapezoidal motion profile constraints
    // Max velocity of 10 meters per second
    // Max acceleration of 20 meters per second squared
    public static final TrapezoidProfile.Constraints PROFILE_CONSTRAITS = 
      new TrapezoidProfile.Constraints(
        10, 20
      );

    public static final double AMP_THRESHOLD = 5;

    public static final double ROT_2_M = 0;

    // The enumeration for the elevator states
    public enum ElevatorState {
      RUN_TO_POSITION,
      RUN_TO_ANGLE,
      RUN_TO_HEIGHT,
      RUN_MANUAL
  }
  }


  public static class ArmConstants {
    public static final double P = 0.217;
    public static final double I = 0.0;
    public static final double D = 0.0;

    public static final TrapezoidProfile.Constraints PROFILE_CONSTRAITS = 
      new TrapezoidProfile.Constraints(
        10, 20
      );

    public static final double MAX_ANGLE = 884000000;
    public static final double MIN_ANGLE = -884000000;

    public enum ArmPosition {
      SOURCE(0),
      REEF(0),
      ALGAE(1);
  
      private final double angle;
  
      ArmPosition(double angle) {
          this.angle = angle;
      }
  
      public double getAngle() {
          return angle;
      }
  }
  
    public static final double ROT_2_RADIAN = 0;
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
