package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ElevatorSensors {
    private final DutyCycleEncoder angleEncoder; // <-- Encoder to measure the angle of the elevator.
    private final TalonFX krakenL, krakenR; // <-- Motor controllers for the left and right sides of the elevator.

    public ElevatorSensors(TalonFX krakenL, TalonFX krakenR) {
        this.krakenL = krakenL; // <-- Assigns the left motor controller.
        this.krakenR = krakenR; // <-- Assigns the right motor controller.
        this.angleEncoder = new DutyCycleEncoder(2); // <-- Initializes the angle encoder on channel 2.
    }

    // Calculates and returns the height of the elevator.
    public double getHeight() {
        double right = krakenR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_M; // <-- Converts right motor rotations to meters.
        double left = krakenL.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_M; // <-- Converts left motor rotations to meters.
        return Math.abs(right - left); // <-- Returns the absolute difference between left and right heights.
    }

    // Calculates and returns the angle of the elevator.
    public double getAngle() {
        double right = krakenR.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_RADIAN; // <-- Converts right motor rotations to radians.
        double left = krakenL.getPosition().getValue().magnitude() * ElevatorConstants.ROT_2_RADIAN; // <-- Converts left motor rotations to radians.
        return Math.abs(right + left); // <-- Returns the absolute sum of left and right angles.
    }
}
