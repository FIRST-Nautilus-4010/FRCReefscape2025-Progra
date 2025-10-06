package frc.robot.subsystems.swerve;

import java.util.Optional;

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
        // Congelar actualizaciones por unos ciclos si hay golpe fuerte
        collisionDetector.freeze();
    } else {
        // Solo actualizar si no hay impacto reciente
        poseEstimator.update(swerve.getRotation2d(), swerve.getSwerveModulePos());
    }

    // ======= Skid detection =======
    confidenceTracker.update(swerve);
    if (confidenceTracker.isSkidding()) {
      // Aumenta la incertidumbre del odómetro temporalmente
      poseEstimator.setVisionMeasurementStdDevs(AutonomousConstants.LOW_CONFIDENCE_STD);
    } else {
      poseEstimator.setVisionMeasurementStdDevs(AutonomousConstants.NORMAL_CONFIDENCE_STD);
    }

    // ======= Vision updates (AprilTags) =======
    var visionMeasurement = getVisionPose(); // retorna Optional<Pose2d>
    if (visionMeasurement.isPresent()) {
      Pose2d visionPose = visionMeasurement.get();
      double timestamp = getLastVisionTimestamp();

      if (confidenceTracker.shouldTrustVision(visionPose, getPose())) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
      }
    }

    // ======= Dashboard & logging =======
    SmartDashboard.putString("Robot Pose", getPose().toString());
    posePublisher.set(getPose());
  }

}
