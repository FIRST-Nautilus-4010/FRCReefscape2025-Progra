package frc.robot.subsystems.swerve;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.commands.DriveTo;

public class PoseTracker {
    

    private final SwerveDriveOdometry odometry;
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Robot position", Pose2d.struct).publish();

    Optional<Alliance> ally = DriverStation.getAlliance();

    private final Swerve swerve;
    private final Vision vision;

    public PoseTracker(Swerve swerve) {
        this.swerve = swerve;
        vision = new Vision();

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
        return new DriveTo(pose, () -> getPose(), swerve);
    }

    public Command rotateTo(Rotation2d angle) {
        return new DriveTo(new Pose2d(
            getPose().getX(), 
            getPose().getY(), 
            angle
        ), () -> getPose(), swerve);
    }

    public void periodic() {
        updateOdometry();
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        posePublisher.set(getPose());
    }
}
