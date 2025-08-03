package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Subsystem responsible for controlling the elevator mechanism
public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io; // Handles inputs and outputs (motors, sensors)
    private final ElevatorController controller; // PID controllers for height and angle
    private final XboxController xboxController; // Operator joystick

    private ElevatorState currentState = ElevatorState.RUN_MANUAL;

    private double targetHeight = 0;
    private double targetAngle = 0;

    public ElevatorSubsystem(XboxController xboxController) {
        this.xboxController = xboxController;
        this.io = new ElevatorIO();
        this.controller = new ElevatorController(io.getLeftLeaderMotor(), io.getRightLeaderMotor());
    }

    public void setState(ElevatorState state) {
        this.currentState = state;
    }

    public void setTargetHeight(double height) {
        targetHeight = height; 
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    // Called periodically: applies control logic based on current elevator state
    @Override
    public void periodic() {
        switch (currentState) {
            case RUN_TO_POSITION:
                controller.setTargetPosition(targetHeight, targetAngle);
                break;
            case RUN_MANUAL:
                double heightPower = xboxController.getRightY(); 
                double anglePower = xboxController.getLeftX();

                controller.setPower(heightPower, anglePower);
                break;
        }

        controller.periodic();
    }
}


