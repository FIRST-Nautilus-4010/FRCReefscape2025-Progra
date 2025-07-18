package frc.robot.subsystems.Elevator;

// Enum to represent the different states of the elevator system.
public enum ElevatorState {
    RUN_TO_POSITION, // <-- State where the elevator moves to a specific position (angle and height).
    RUN_TO_ANGLE,    // <-- State where the elevator adjusts to a specific angle.
    RUN_TO_HEIGHT,   // <-- State where the elevator adjusts to a specific height.
    RUN_MANUAL       // <-- State where the elevator is controlled manually by the operator.
}