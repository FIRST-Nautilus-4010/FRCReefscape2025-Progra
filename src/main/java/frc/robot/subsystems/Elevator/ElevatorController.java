package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;

/**
 * Handles the control logic for the elevator's height and angle using Profiled PID controllers.
 */
public class ElevatorController {
    private final ProfiledPIDController anglePID;
    private final ProfiledPIDController heightPID;

    /**
     * Constructs the ElevatorController and initializes both angle and height PID controllers
     * using the constants defined in {@link ElevatorConstants}.
     */
    public ElevatorController() {
        anglePID = new ProfiledPIDController(
            ElevatorConstants.P_ANGLE,
            ElevatorConstants.I_ANGLE,
            ElevatorConstants.D_ANGLE,
            ElevatorConstants.ANGLE_CONSTRAINTS
        );
        // Allow angle values to wrap around (e.g., from π to -π), useful for rotational control.
        anglePID.enableContinuousInput(-Math.PI, Math.PI);

        heightPID = new ProfiledPIDController(
            ElevatorConstants.P_ELEVATOR,
            ElevatorConstants.I_ELEVATOR,
            ElevatorConstants.D_ELEVATOR,
            ElevatorConstants.HEIGHT_CONSTRAINTS
        );
    }

    /**
     * Calculates the output needed to reach the target angle from the current angle.
     *
     * @param currentAngle Current measured angle.
     * @return Motor output to reach the target angle.
     */
    public double calculateAngle(double currentAngle) {
        return anglePID.calculate(currentAngle);
    }

    /**
     * Calculates the output needed to reach the target height from the current height.
     *
     * @param currentHeight Current measured height.
     * @return Motor output to reach the target height.
     */
    public double calculateHeight(double currentHeight) {
        return heightPID.calculate(currentHeight);
    }

    /**
     * Sets a new target angle for the angle PID controller.
     *
     * @param angle Target angle in radians.
     */
    public void setTargetAngle(double angle) {
        anglePID.setGoal(angle);
    }

    /**
     * Sets a new target height for the height PID controller.
     *
     * @param height Target height in meters.
     */
    public void setTargetHeight(double height) {
        heightPID.setGoal(height);
    }
}
