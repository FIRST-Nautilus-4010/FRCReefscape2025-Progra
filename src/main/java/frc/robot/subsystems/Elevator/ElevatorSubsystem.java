package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Subsystem responsible for controlling the elevator mechanism
public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io; // Handles motor control for the elevator
    private final ElevatorSensors sensors; // Provides height and angle feedback
    private final ElevatorController controller; // PID controllers for height and angle
    private final XboxController xboxController; // Operator joystick

    private ElevatorState currentState = ElevatorState.RUN_MANUAL;

    public ElevatorSubsystem(XboxController xboxController) {
        this.xboxController = xboxController;
        this.io = new ElevatorIO();
        this.sensors = new ElevatorSensors(io.getLeftMotor(), io.getRightMotor());
        this.controller = new ElevatorController();
    }

    public void setState(ElevatorState state) {
        this.currentState = state;
    }

    public void setTargetHeight(double height) {
        controller.setTargetHeight(height);
    }

    public void setTargetAngle(double angle) {
        controller.setTargetAngle(angle);
    }

    // Called periodically: applies control logic based on current elevator state
    @Override
    public void periodic() {
        double heightPower = 0, anglePower = 0;

        switch (currentState) {
            case RUN_TO_POSITION:
                heightPower = controller.calculateHeight(sensors.getHeight());
                anglePower = controller.calculateAngle(sensors.getAngle());
                break;
            case RUN_TO_HEIGHT:
                heightPower = controller.calculateHeight(sensors.getHeight());
                anglePower = xboxController.getLeftX(); // manual angle control
                break;
            case RUN_TO_ANGLE:
                anglePower = controller.calculateAngle(sensors.getAngle());
                heightPower = xboxController.getRightY(); // manual height control
                break;
            case RUN_MANUAL:
                heightPower = xboxController.getRightY(); 
                anglePower = xboxController.getLeftX();
                break;
        }

        // Combine height and angle control using differential motor power
        io.setMotorPowers(heightPower + anglePower, heightPower - anglePower);
    }
}


