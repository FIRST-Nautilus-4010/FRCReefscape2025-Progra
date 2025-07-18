package frc.robot.subsystems.Elevator;

public enum PredefinedElevatorPositions {
    SOURCE(0, 1), REEF(0, 1), ALGAE(1, 1);

    private final double angle;
    private final double height;

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

