package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.PoseTracker;

public class DriveToCoral extends Command {
    private final Swerve swerve;
    private final PoseTracker poseTracker;
    private DriveTo driveToCommand;

    private final StructPublisher<Pose2d> targetCoralPublisher = 
    NetworkTableInstance.getDefault()
    .getStructTopic("Target coral", Pose2d.struct)
    .publish();

    public DriveToCoral(Swerve swerve, PoseTracker poseTracker) {
        this.swerve = swerve;
        this.poseTracker = poseTracker;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Reutilizar DriveTo con ese target dinámico
        driveToCommand = new DriveTo(getClosestCoralPos(), swerve, poseTracker);
        driveToCommand.initialize();
    }

    @Override
    public void execute() {
        Pose2d newTarget = getClosestCoralPos();
        if (!newTarget.equals(driveToCommand.getTargetPose())) {
            driveToCommand = new DriveTo(newTarget, swerve, poseTracker);
            driveToCommand.initialize();
        }
        driveToCommand.execute();
        
    }

    @Override
    public void end(boolean interrupted) {
        if (driveToCommand != null) driveToCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    public Pose2d getClosestCoralPos() {
        double closestCoralDistance = 100000;
        Pose2d closestCoralPos = null;

        for (Pose2d coral : poseTracker.getCoralPoses()) {
            double tmpCoralDistance = coral.getTranslation().getDistance(poseTracker.getPose().getTranslation());
            if (tmpCoralDistance < closestCoralDistance) {
                closestCoralDistance = tmpCoralDistance;
                closestCoralPos = coral;
            }
        }

        targetCoralPublisher.set(closestCoralPos);

        return closestCoralPos;
    }
}
