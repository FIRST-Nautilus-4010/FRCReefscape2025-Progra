package frc.robot.subsystems.swerve.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.PoseTracker;
import frc.robot.utils.LimelightHelpers;

public class AssistToCoral extends Command {
    private final Swerve swerve;
    private final PoseTracker poseTracker;
    private final DoubleSupplier xInput, yInput, rotInput;
    private final double kAssist = 0.15;
    private final double maxAssist = 1.0;

    public AssistToCoral(Swerve swerve, PoseTracker poseTracker,
                         DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rotInput) {
        this.swerve = swerve;
        this.poseTracker = poseTracker;
        this.xInput = xInput;
        this.yInput = yInput;
        this.rotInput = rotInput;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double tx = LimelightHelpers.getTX("limelight-coral");
        double ty = LimelightHelpers.getTY("limelight-coral");
        if (LimelightHelpers.getTargetCount("limelight-coral") == 0) {
            swerve.driveFieldRelative(xInput.getAsDouble(), yInput.getAsDouble(), rotInput.getAsDouble());
            return;
        }

        // Calcula vector hacia el coral
        double dist = estimateDistanceFromTY(ty);
        Translation2d toCoral = new Translation2d(dist * Math.cos(Math.toRadians(tx)), dist * Math.sin(Math.toRadians(tx)));
        Translation2d coralDir = toCoral.div(toCoral.getNorm());
        Translation2d coralPerp = new Translation2d(-coralDir.getY(), coralDir.getX());

        // Entrada del driver
        Translation2d driverVec = new Translation2d(xInput.getAsDouble(), yInput.getAsDouble());

        // Error lateral
        double lateralError = toCoral.getX() * coralPerp.getX() + toCoral.getY() * coralPerp.getY();
        double assistMag = Math.max(-maxAssist, Math.min(maxAssist, kAssist * lateralError));

        Translation2d finalVec = driverVec.plus(coralPerp.times(assistMag));
        swerve.driveFieldRelative(finalVec.getX(), finalVec.getY(), rotInput.getAsDouble());
    }

    private double estimateDistanceFromTY(double ty) {
        double cameraHeight = 0.6;
        double targetHeight = 1.1;
        double cameraAngle = Math.toRadians(30);
        return (targetHeight - cameraHeight) / Math.tan(cameraAngle + Math.toRadians(ty));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
