/*package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.utils.Camera;
import frc.robot.utils.Constants;

public class ApriltagsExample extends Command{
    ArrayList<Camera> cameras;

    @Override
    public void initialize() {
        for (int i = 0; i < Constants.VisionConstants.ROBOT_TO_CAM.length; i++) {
            cameras.add(new Camera(i, Constants.VisionConstants.ROBOT_TO_CAM[i]));
        }
    }
    
    @Override
    public void execute() {
        for (Camera camera : cameras) {
            if (camera.hasApriltags()) {
                Object tvec = camera.getApriltags().get("tvec");
                Object rvec = camera.getApriltags().get("rvec");

                if (tvec instanceof List<?> && rvec instanceof List<?>){
                    List<?> tvecList = (List<?>) tvec;
                    List<?> rvecList = (List<?>) rvec;

                    if (tvecList.get(0) instanceof List<?> && rvecList.get(0) instanceof List<?>) {
                        List<Double> tvalues = (List<Double>) tvecList.get(0);
                        List<Double> rvalues = (List<Double>) rvecList.get(0);

                        Transform3d apriltagToCam = new Transform3d(
                            new Translation3d(tvalues.get(0), tvalues.get(1), tvalues.get(2)), 
                            new Rotation3d(rvalues.get(0), rvalues.get(1), rvalues.get(2))
                        );

                        Transform3d apriltagPos = camera.positionTorobot.plus(apriltagToCam);
                        
                        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutonomousConstants.MAX_SPD, AutonomousConstants.MAX_ACCEL);

                        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            RobotContainer.swerve.getPose(),
                            List.of(apriltagPos.getTranslation().toTranslation2d().minus(new Translation2d(.5, .5)).div(2)),
                            new Pose2d(apriltagPos.getTranslation().toTranslation2d().minus(new Translation2d(.5, .5)), new Rotation2d(Math.atan2(apriltagPos.getY(), apriltagPos.getX()))),
                            trajectoryConfig
                            
                        );
                        
                        PIDController xController = new PIDController(AutonomousConstants.P, AutonomousConstants.I, AutonomousConstants.D);
                        PIDController yController = new PIDController(AutonomousConstants.P, AutonomousConstants.I, AutonomousConstants.D);
                        ProfiledPIDController zController = new ProfiledPIDController(AutonomousConstants.P_Z, 
                            AutonomousConstants.I_Z, AutonomousConstants.D_Z, AutonomousConstants.Z_CONTROLER
                        );
                        zController.enableContinuousInput(-Math.PI, Math.PI);

                        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
                            RobotContainer.swerve::getPose,
                            Constants.ChassisConstants.KINEMATICS,
                            xController,
                            yController,
                            zController,
                            RobotContainer.swerve::setStates,
                            RobotContainer.swerve
                        );

                        swerveControllerCommand.initialize();
                        swerveControllerCommand.execute();
                        swerveControllerCommand.end(isFinished());

                        break;
                    }
                }
            }
        }
    }
}*/
