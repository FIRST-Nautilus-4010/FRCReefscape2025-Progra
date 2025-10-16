package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.PoseTracker;
import frc.robot.utils.LimelightHelpers;

public class DriveToCoral extends Command {
    private final Swerve swerve;
    private final PoseTracker poseTracker;
    private DriveTo driveToCommand;

    public DriveToCoral(Swerve swerve, PoseTracker poseTracker) {
        this.swerve = swerve;
        this.poseTracker = poseTracker;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (LimelightHelpers.getTargetCount("limelight-coral") == 0) {
            cancel();
            return;
        }

        // Calcular la posición del coral detectado
        double tx = LimelightHelpers.getTX("limelight-coral");
        double ty = LimelightHelpers.getTY("limelight-coral");
        double distance = estimateDistanceFromTY(ty);

        // Coral en coordenadas relativas al robot
        double coralX = distance * Math.cos(Math.toRadians(tx));
        double coralY = distance * Math.sin(Math.toRadians(tx));

        Pose2d robotPose = poseTracker.getPose();
        Translation2d coralField = robotPose.getTranslation().plus(new Translation2d(coralX, coralY));

        Pose2d targetPose = new Pose2d(coralField, robotPose.getRotation());

        // Reutilizar DriveTo con ese target dinámico
        driveToCommand = new DriveTo(targetPose, swerve, poseTracker);
        driveToCommand.initialize();
    }

    @Override
    public void execute() {
        if (driveToCommand != null) driveToCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (driveToCommand != null) driveToCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return driveToCommand == null || driveToCommand.isFinished();
    }

    private double estimateDistanceFromTY(double ty) {
        double cameraHeight = 0.6;  // m
        double targetHeight = 1.1;  // m
        double cameraAngle = Math.toRadians(30);
        return (targetHeight - cameraHeight) / Math.tan(cameraAngle + Math.toRadians(ty));
    }
}
