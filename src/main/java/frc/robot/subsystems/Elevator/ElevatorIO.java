package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIO {
    // Motor controllers for the elevator system.
    private final TalonFX krakenLBottom = new TalonFX(16); // <-- Left bottom motor.
    private final TalonFX krakenLTop = new TalonFX(15); // <-- Left top motor (follower).
    private final TalonFX krakenRBottom = new TalonFX(17); // <-- Right bottom motor.
    private final TalonFX krakenRTop = new TalonFX(18); // <-- Right top motor (follower).

    public ElevatorIO() {
        // Configures the top motors to follow the bottom motors.
        krakenLTop.setControl(new Follower(krakenLBottom.getDeviceID(), false)); // <-- Left top motor follows left bottom motor.
        krakenRTop.setControl(new Follower(krakenRBottom.getDeviceID(), false)); // <-- Right top motor follows right bottom motor.
    }

    // Returns the left bottom motor controller.
    public TalonFX getLeftMotor() {
        return krakenLBottom; // <-- Provides access to the left bottom motor.
    }

    // Returns the right bottom motor controller.
    public TalonFX getRightMotor() {
        return krakenRBottom; // <-- Provides access to the right bottom motor.
    }

    // Sets the power for both left and right bottom motors.
    public void setMotorPowers(double left, double right) {
        krakenLBottom.set(left); // <-- Sets the power for the left bottom motor.
        krakenRBottom.set(right); // <-- Sets the power for the right bottom motor.
    }
}
