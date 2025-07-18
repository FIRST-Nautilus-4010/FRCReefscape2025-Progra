package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io; // <-- Handles motor control for the elevator.
    private final ElevatorSensors sensors; // <-- Provides sensor feedback for the elevator's position and angle.
    private final ElevatorController controller; // <-- Manages PID control for height and angle adjustments.
    private final Joystick joystick; // <-- Joystick for manual control of the elevator.

    private ElevatorState currentState = ElevatorState.RUN_MANUAL; // <-- Initial state set to manual control.

    public ElevatorSubsystem(Joystick joystick) {
        this.joystick = joystick; // <-- Assigns the joystick for operator control.
        this.io = new ElevatorIO(); // <-- Initializes the motor controllers.
        this.sensors = new ElevatorSensors(io.getLeftMotor(), io.getRightMotor()); // <-- Initializes sensors using the motor controllers.
        this.controller = new ElevatorController(); // <-- Initializes the PID controllers for precise control.
    }

    // Updates the current state of the elevator system.
    public void setState(ElevatorState state) {
        this.currentState = state; // <-- Sets the elevator's operational state.
    }

    // Sets the target height for the elevator.
    public void setTargetHeight(double height) {
        controller.setTargetHeight(height); // <-- Passes the desired height to the height PID controller.
    }

    // Sets the target angle for the elevator.
    public void setTargetAngle(double angle) {
        controller.setTargetAngle(angle); // <-- Passes the desired angle to the angle PID controller.
    }

    // Periodic method to execute logic based on the current state of the elevator.
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
                anglePower = joystick.getX();
                break;
            case RUN_TO_ANGLE:
                anglePower = controller.calculateAngle(sensors.getAngle());
                heightPower = joystick.getY();
                break;
            case RUN_MANUAL:
            default:
                heightPower = joystick.getY();
                anglePower = joystick.getX();
        }

        io.setMotorPowers(heightPower + anglePower, heightPower - anglePower);
    }
}

