package frc.robot.subsystems.swerve.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.PoseTracker;
import frc.robot.utils.LimelightHelpers;

public class AssistToCoral extends Command {
    private final Swerve swerve;
    private final PoseTracker poseTracker;
    private final Supplier<Double> xInput, yInput, rotInput;
    private final Supplier<Boolean> hasIntake;
    private final double kAssist = 0.15;
    private final double maxAssist = 1.0;

    public AssistToCoral(Swerve swerve, PoseTracker poseTracker,
    Supplier<Double> xInput, Supplier<Double> yInput, Supplier<Double> rotInput, Supplier<Boolean> hasIntake) {
        this.swerve = swerve;
        this.poseTracker = poseTracker;
        this.xInput = xInput;
        this.yInput = yInput;
        this.rotInput = rotInput;
        this.hasIntake = hasIntake;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Pose2d closestCoral = getClosestCoralAnglePos();
        double relativeAngle = 0;

        if (closestCoral == null) {
            swerve.driveFieldRelative(xInput.get(), yInput.get(), rotInput.get());
            return;
        } else {
            double absoluteCoralAngle = Math.atan2(closestCoral.getY() - poseTracker.getPose().getY(),  closestCoral.getX() - poseTracker.getPose().getX());
            double absoluteVelocityAngle = Math.atan2(yInput.get(), xInput.get());
            relativeAngle = absoluteCoralAngle - absoluteVelocityAngle;

            if (Math.abs(relativeAngle) > Math.toRadians(15)) {
                swerve.driveFieldRelative(xInput.get(), yInput.get(), rotInput.get());
                return;
            }
        }

        double distance = closestCoral.getTranslation().getDistance(poseTracker.getPose().getTranslation());

        double finalVecX = Math.cos(relativeAngle + Math.PI) * distance * Math.sin(relativeAngle);
        double finalVecY = Math.sin(relativeAngle + Math.PI) * distance * Math.sin(relativeAngle);

        finalVecX += xInput.get();
        finalVecY += yInput.get();

        swerve.driveFieldRelative(finalVecX, finalVecY, rotInput.get());
    }
    @Override
    public boolean isFinished() {
        return hasIntake.get();
    }

    public Pose2d getClosestCoralAnglePos() {
        double closestAngle = 100000;
        Pose2d closestCoralPos = null;

        for (Pose2d coral : poseTracker.getCoralPoses()){
            double absoluteCoralAngle = Math.atan2(coral.getY(), coral.getX());
            double absoluteVelocityAngle = Math.atan2(yInput.get(), xInput.get());
            double relativeAngle = absoluteCoralAngle - absoluteVelocityAngle;

            if (Math.abs(relativeAngle) < Math.abs(closestAngle)) {
                closestAngle = relativeAngle;
                closestCoralPos = coral;
            }
        }

        return closestCoralPos;
    }
}
