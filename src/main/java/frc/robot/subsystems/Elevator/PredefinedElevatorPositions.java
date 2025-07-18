package frc.robot.subsystems.Elevator;

/**
 * Enum representing predefined target positions for the elevator mechanism.
 * Each position includes a target angle and height.
 */
public enum PredefinedElevatorPositions {
    SOURCE(0, 1),
    REEF(0, 1),
    ALGAE(1, 1);

    private final double angle;
    private final double height;

    /**
     * Constructs a predefined elevator position.
     *
     * @param angle  The target angle (radians or degrees, depending on system).
     * @param height The target height (in meters).
     */
    PredefinedElevatorPositions(double angle, double height) {
        this.angle = angle;
        this.height = height;
    }

    public double getAngle() {
        return angle;
    }

    public double getHeight() {
        return height;
    }
}
