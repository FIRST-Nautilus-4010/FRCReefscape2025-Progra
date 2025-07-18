package frc.robot.subsystems.Elevator;

// Enum to define predefined positions for the elevator system.
public enum PredefinedElevatorPositions {
    SOURCE(0, 1), // <-- Position for the "SOURCE" target with specific angle and height.
    REEF(0, 1),   // <-- Position for the "REEF" target with specific angle and height.
    ALGAE(1, 1);  // <-- Position for the "ALGAE" target with specific angle and height.

    private final double angle;  // <-- Angle of the elevator for the predefined position.
    private final double height; // <-- Height of the elevator for the predefined position.

    // Constructor to initialize the angle and height for each predefined position.
    PredefinedElevatorPositions(double angle, double height) {
        this.angle = angle; // <-- Assigns the angle for the position.
        this.height = height; // <-- Assigns the height for the position.
    }

    // Returns the angle of the predefined position.
    public double getAngle() {
        return angle; // <-- Provides access to the angle.
    }

    // Returns the height of the predefined position.
    public double getHeight() {
        return height; // <-- Provides access to the height.
    }
}
