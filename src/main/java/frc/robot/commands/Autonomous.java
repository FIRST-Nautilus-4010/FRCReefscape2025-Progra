package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.utils.Constants.AutonomousConstants;
import frc.robot.utils.Constants.ChassisConstants;

public class Autonomous extends Command {
    private static final double LARGE_DISTANCE = 1000000;
    private Pose3d focusedReef = new Pose3d(new Translation3d(LARGE_DISTANCE, LARGE_DISTANCE, 0), new Rotation3d());
    private Pose3d focusedSource = new Pose3d(new Translation3d(LARGE_DISTANCE, LARGE_DISTANCE, 0), new Rotation3d());
    private int reefId = 0;
    private int sourceId = 0;
    private List<Boolean> visitedReefs;
    private boolean hasAlgae = false;

    public Autonomous() {
        visitedReefs = new ArrayList<>(Collections.nCopies(AutonomousConstants.REEF_POS.length, false));
    }

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

                if (currentDistance < focusedDistance && !visitedReefs.get(i)) {
                    focusedReef = AutonomousConstants.REEF_POS[i];
                    focusedSource = AutonomousConstants.SOURCE_POS[j];
                    reefId = i;
                    sourceId = j;
                    visitedReefs.set(i, true);
                }
            }
        }

        if (reefId / 2 == Math.round(reefId /  2)) {
            hasAlgae = visitedReefs.get(reefId) || visitedReefs.get(reefId + 1);
        } else {
            hasAlgae = visitedReefs.get(reefId) || visitedReefs.get(reefId - 1);
        }

        // Move to the source
        RobotContainer.goToSource(sourceId);

        // If coral is detected, move to the reef
        if (RobotContainer.hasCoral()) {
            if (hasAlgae) {
                RobotContainer.removeAlgae((int) sourceId / 2);
            }
            
            RobotContainer.goToReef(reefId);
        }
            
    }
}
