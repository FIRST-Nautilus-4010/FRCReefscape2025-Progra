package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.Constants.AutonomousConstants;

public class Autonomous extends Command {
    private static final double LARGE_DISTANCE = 1000000;
    private Pose3d focusedReef = new Pose3d(new Translation3d(LARGE_DISTANCE, LARGE_DISTANCE, 0), new Rotation3d());
    private Pose3d focusedSource = new Pose3d(new Translation3d(LARGE_DISTANCE, LARGE_DISTANCE, 0), new Rotation3d());
    private int reefId = 0;
    private int sourceId = 0;

    @Override
    public void initialize() {
        RobotContainer.claw.calibrateClaw();
    }

    @Override
    public void execute() {
        // Find the closest reef and source pair
        for (int i = 0; i < AutonomousConstants.REEF_POS.length; i++) {
            for (int j = 0; j < AutonomousConstants.SOURCE_POS.length; j++) {
                double currentDistance = AutonomousConstants.SOURCE_POS[j].toPose2d().getTranslation()
                        .getDistance(AutonomousConstants.REEF_POS[i].toPose2d().getTranslation())
                        - AutonomousConstants.REEF_POS[i].getZ();
                double focusedDistance = focusedSource.toPose2d().getTranslation()
                        .getDistance(focusedReef.toPose2d().getTranslation())
                        - focusedReef.getZ();

                if (currentDistance < focusedDistance) {
                    focusedReef = AutonomousConstants.REEF_POS[i];
                    focusedSource = AutonomousConstants.SOURCE_POS[j];
                    reefId = i;
                    sourceId = j;
                }
            }
        }

        // Move to the source
        RobotContainer.goToSource(sourceId);

        // If coral is detected, move to the reef
        if (RobotContainer.hasCoral()) {
            RobotContainer.goToReef(reefId);
        }
    }
}
