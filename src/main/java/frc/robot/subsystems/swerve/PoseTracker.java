package frc.robot.subsystems.swerve;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Vision;

public class PoseTracker {
    private final TrajectoryConfig trajectoryConfig;
    private final HolonomicDriveController controller;

    private final SwerveDriveOdometry odometry;
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Robot position", Pose2d.struct).publish();

    Optional<Alliance> ally = DriverStation.getAlliance();

    private final Swerve swerve;
    private final Vision vision;

    public PoseTracker(Swerve swerve) {
        this.swerve = swerve;
        vision = new Vision();
        
        ProfiledPIDController thetaController = new ProfiledPIDController(AutonomousConstants.P_Z, AutonomousConstants.I_Z, AutonomousConstants.D_Z, AutonomousConstants.Z_CONTROLER);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        controller = new HolonomicDriveController(
            new PIDController(AutonomousConstants.P_X, AutonomousConstants.I_X, AutonomousConstants.D_X),
            new PIDController(AutonomousConstants.P_Y, AutonomousConstants.I_Y, AutonomousConstants.D_Y),
            thetaController
        );

        trajectoryConfig = new TrajectoryConfig(AutonomousConstants.MAX_SPD, AutonomousConstants.MAX_ACCEL)
            .setKinematics(ChassisConstants.KINEMATICS);

        odometry = new SwerveDriveOdometry(ChassisConstants.KINEMATICS,
            new Rotation2d(0), 
            swerve.getSwerveModulePos(),
            AutonomousConstants.initialPose
        );
    }

    // Return actual robot position
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // Resets the odometry
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(swerve.getRotation2d(), swerve.getSwerveModulePos(), pose);
    }

    // Updates the odometry
    private void updateOdometry(){
        boolean isRed;
        if (!ally.isEmpty()){
            isRed = ally.get() == Alliance.Red;
        } else {
            isRed = true;
        }

        if (vision.getAprilDetections("limelight-three") > 0){
            resetOdometry(
                vision.getRobotPoseFromAprilTags(
                    swerve.getHeading(), 
                    swerve.getPitch(), 
                    swerve.getRoll(),
                    isRed
                ).pose
            );
            return;
        }
        odometry.update(swerve.getRotation2d(), swerve.getSwerveModulePos());
    }

    public Command driveTo(Pose2d pose) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            getPose(),
            List.of(),
            pose,
            trajectoryConfig
        );
        
        return new SwerveControllerCommand(trajectory, this::getPose, ChassisConstants.KINEMATICS, controller, swerve::setStates, swerve);
    }

    public Command rotateTo(Rotation2d angle) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            getPose(),
            List.of(),
            new Pose2d(getPose().getTranslation(), angle),
            trajectoryConfig
        );

        return new SwerveControllerCommand(trajectory, this::getPose, ChassisConstants.KINEMATICS, controller, swerve::setStates, swerve);
    }

    public void periodic() {
        updateOdometry();
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        posePublisher.set(getPose());
    }
}
