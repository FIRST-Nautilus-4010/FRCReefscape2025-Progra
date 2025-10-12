package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.swerve.commands.AssistToCoral;
import frc.robot.subsystems.swerve.commands.DriveTo;
import frc.robot.subsystems.swerve.commands.DriveToCoral;
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
    private final StructArrayPublisher<Pose2d> coralPublisher = 
        NetworkTableInstance.getDefault()
        .getStructArrayTopic("Detected corals", Pose2d.struct)
        .publish();

    private List<Pose2d> coralPoses = new ArrayList<>();

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

    public Command setSpeeds(
        Supplier<Double> vx, Supplier<Double> vy, 
        Supplier<Double> omega, Supplier<Boolean> fieldRelative, 
        Supplier<Boolean> resetYaw, boolean assist, Supplier<Boolean> hasIntake
    ) {
        if (assist) {
            return new AssistToCoral(swerve, this, vx, vy, omega, hasIntake);
        }
        return new SwerveDriveJoystick(swerve, vx, vy, omega, fieldRelative, resetYaw);
    }

    public Command driveToCoral() {
        return new DriveToCoral(swerve, this);
    }

    private Optional<Pose2d> getVisionPose() {
        if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getTA("limelight") > 0.1) {
            LimelightHelpers.SetRobotOrientation("limelight", 
            swerve.getHeading(), swerve.getYawRate(), swerve.getPitch(), swerve.getPitchRate(), swerve.getRoll(), swerve.getRollRate());
            Pose2d botpose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            return Optional.of(botpose);
        }
        return Optional.empty();
    }

    private double getLastVisionTimestamp() {
        return Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture("limelight") / 1000.0;
    }

    private void detectCorals() {
        List<Pose2d> detectedCorals = new ArrayList<>();

        var table = NetworkTableInstance.getDefault().getTable("limelight-coral");

        double[] txs = table.getEntry("tx").getDoubleArray(new double[0]);
        double[] tys = table.getEntry("ty").getDoubleArray(new double[0]);

        for (int i = 0; i < LimelightHelpers.getTargetCount("limelight-coral"); i++) {
            // Calcular la posición del coral detectado
            double tx = txs[i];
            double ty = tys[i];

            double cameraHeight = 0.5; // Altura de la cámara en metros
            double cameraXOffset = 0.0; // Desplazamiento horizontal de la cámara desde el centro del robot en metros
            double cameraYOffset = 0.0; // Desplazamiento vertical de la cámara desde el centro del robot en metros
            double cameraPitch = 35; // Ángulo de inclinación de la cámara en grados
            double cameraYaw = 0; // Ángulo de giro de la cámara en grados
            
            Pose2d botPos = getPose();

            double distance = cameraHeight / Math.tan(Math.toRadians(-ty + cameraPitch));

            double y = distance * Math.cos(Math.toRadians(tx + cameraYaw + botPos.getRotation().getDegrees())) + cameraYOffset;
            double x = distance * Math.sin(Math.toRadians(tx + cameraYaw + botPos.getRotation().getDegrees())) + cameraXOffset;

            y += botPos.getY();
            x += botPos.getX();

            detectedCorals.add(new Pose2d(new Translation2d(x, y), new Rotation2d()));
        }

        coralPoses = detectedCorals;
        coralPublisher.set(coralPoses.toArray(new Pose2d[0]));
    }

    public Pose2d[] getCoralPoses() {
        return coralPoses.toArray(new Pose2d[0]);
    }

    public int getCoralCount() {
        return coralPoses.size();
    }

    double lastDebugTime = 0;

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
        
        if (Timer.getFPGATimestamp() - lastDebugTime > 0.2) {
            lastDebugTime = Timer.getFPGATimestamp();
            SmartDashboard.putBoolean("Is Skidding", confidenceTracker.isSkidding());
            detectCorals();
        }
    }

}
