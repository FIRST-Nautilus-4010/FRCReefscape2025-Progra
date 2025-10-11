package frc.robot.subsystems.swerve;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.swerve.commands.DriveTo;
import frc.robot.subsystems.swerve.commands.SwerveDriveJoystick;
import frc.robot.utils.PoseConfidenceTracker; // te explico más abajo
import frc.robot.utils.CollisionDetector;      // idem
import frc.robot.utils.LimelightHelpers;

public class PoseTracker {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final StructPublisher<Pose2d> posePublisher = 
        NetworkTableInstance.getDefault()
        .getStructTopic("Robot position", Pose2d.struct)
        .publish();

    private final Swerve swerve;

    // utilidades nuevas
    private final PoseConfidenceTracker confidenceTracker = new PoseConfidenceTracker();
    private final CollisionDetector collisionDetector = new CollisionDetector();

    private boolean initialPoseSetFromVision = false;

    public PoseTracker(Swerve swerve) {
        this.swerve = swerve;

        poseEstimator = new SwerveDrivePoseEstimator(
            ChassisConstants.KINEMATICS,
            new Rotation2d(0),
            swerve.getSwerveModulePos(),
            AutonomousConstants.initialPose
        );
    }

     // === Métodos principales ===

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(swerve.getRotation2d(), swerve.getSwerveModulePos(), pose);
    }

    public Command driveTo(Pose2d pose) {
        return new DriveTo(pose, swerve, this);
    }

    public Command rotateTo(Rotation2d angle) {
        return new DriveTo(
            new Pose2d(getPose().getX(), getPose().getY(), angle),
            swerve, this
        );
    }

    public Command setSpeeds(Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative, Supplier<Boolean> resetYaw) {
        return new SwerveDriveJoystick(swerve, vx, vy, omega, fieldRelative, resetYaw);
    }

    private Optional<Pose2d> getVisionPose() {
        if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getTA("limelight") > 0.1) {
            Pose2d botpose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            return Optional.of(botpose);
        }
        return Optional.empty();
    }

    private double getLastVisionTimestamp() {
        return Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture("limelight") / 1000.0;
    }

  public void periodic() {
     // ======= Collision detection =======
     if (collisionDetector.detectImpact(swerve)) {
        collisionDetector.freeze();
    } else {
        poseEstimator.update(swerve.getRotation2d(), swerve.getSwerveModulePos());
    }

    // ======= Skid detection =======
    confidenceTracker.update(swerve);
    if (confidenceTracker.isSkidding()) {
        poseEstimator.setVisionMeasurementStdDevs(AutonomousConstants.LOW_CONFIDENCE_STD);
    } else {
        poseEstimator.setVisionMeasurementStdDevs(AutonomousConstants.NORMAL_CONFIDENCE_STD);
    }

    // ======= Vision updates (AprilTags / Limelight) =======
    var visionMeasurement = getVisionPose();
    if (visionMeasurement.isPresent()) {
        Pose2d visionPose = visionMeasurement.get();
        double timestamp = getLastVisionTimestamp();

        // Si aún no hemos fijado la posición inicial con visión
        if (!initialPoseSetFromVision) {
            resetOdometry(visionPose);  // Reinicia usando la pose de la cámara
            initialPoseSetFromVision = true;
            SmartDashboard.putString("Init Pose Source", "Limelight");
        }

        // Luego sigue con el procesamiento normal de visión
        if (confidenceTracker.shouldTrustVision(visionPose, getPose())) {
            poseEstimator.addVisionMeasurement(visionPose, timestamp);
        }
    }

    // ======= Dashboard =======
    SmartDashboard.putString("Robot Pose", getPose().toString());
    posePublisher.set(getPose());
  }

}
