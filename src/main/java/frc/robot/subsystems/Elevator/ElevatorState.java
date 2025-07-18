package frc.robot.subsystems.Elevator;

/**
 * Represents the different operational states of the elevator system.
 */
public enum ElevatorState {
    /** Automatically move to a predefined position (angle and height). */
    RUN_TO_POSITION,

    /** Automatically adjust only the elevator angle. */
    RUN_TO_ANGLE,

    /** Automatically adjust only the elevator height. */
    RUN_TO_HEIGHT,

    /** Manually control both height and angle via joystick input. */
    RUN_MANUAL
}
