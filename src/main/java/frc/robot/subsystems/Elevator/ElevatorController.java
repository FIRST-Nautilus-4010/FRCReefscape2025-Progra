package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;

public class ElevatorController {
    private final ProfiledPIDController anglePID;
    private final ProfiledPIDController heightPID;

    public ElevatorController() {
        anglePID = new ProfiledPIDController(ElevatorConstants.P_ANGLE, ElevatorConstants.I_ANGLE, ElevatorConstants.D_ANGLE, ElevatorConstants.ANGLE_CONSTRAINTS);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);

        heightPID = new ProfiledPIDController(ElevatorConstants.P_ELEVATOR, ElevatorConstants.I_ELEVATOR, ElevatorConstants.D_ELEVATOR, ElevatorConstants.HEIGHT_CONSTRAINTS);
    }

    public double calculateAngle(double currentAngle) {
        return anglePID.calculate(currentAngle);
    }

    public double calculateHeight(double currentHeight) {
        return heightPID.calculate(currentHeight);
    }

    public void setTargetAngle(double angle) {
        anglePID.setGoal(angle);
    }

    public void setTargetHeight(double height) {
        heightPID.setGoal(height);
    }
}

