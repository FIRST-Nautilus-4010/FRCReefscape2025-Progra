package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;

public class ElevatorController {
    // PID controllers for controlling the angle and height of the elevator.
    private final ProfiledPIDController anglePID;
    private final ProfiledPIDController heightPID;

    // Constructor to initialize the PID controllers with constants from ElevatorConstants.
    public ElevatorController() {
        // Initializes the angle PID controller with proportional, integral, and derivative gains,
        // as well as motion constraints for the angle.
        anglePID = new ProfiledPIDController(
            ElevatorConstants.P_ANGLE, // <-- Proportional gain for angle control.
            ElevatorConstants.I_ANGLE, // <-- Integral gain for angle control.
            ElevatorConstants.D_ANGLE, // <-- Derivative gain for angle control.
            ElevatorConstants.ANGLE_CONSTRAINTS // <-- Motion constraints for angle control.
        );
        anglePID.enableContinuousInput(-Math.PI, Math.PI); // <-- Enables continuous input for angles between -π and π.

        // Initializes the height PID controller with proportional, integral, and derivative gains,
        // as well as motion constraints for the height.
        heightPID = new ProfiledPIDController(
            ElevatorConstants.P_ELEVATOR, // <-- Proportional gain for height control.
            ElevatorConstants.I_ELEVATOR, // <-- Integral gain for height control.
            ElevatorConstants.D_ELEVATOR, // <-- Derivative gain for height control.
            ElevatorConstants.HEIGHT_CONSTRAINTS // <-- Motion constraints for height control.
        );
    }

    // Calculates the output for the angle PID controller based on the current angle.
    public double calculateAngle(double currentAngle) {
        return anglePID.calculate(currentAngle);
    }

    // Calculates the output for the height PID controller based on the current height.
    public double calculateHeight(double currentHeight) {
        return heightPID.calculate(currentHeight);
    }

    // Sets the target angle for the angle PID controller.
    public void setTargetAngle(double angle) {
        anglePID.setGoal(angle);
    }

    // Sets the target height for the height PID controller.
    public void setTargetHeight(double height) {
        heightPID.setGoal(height);
    }
}
