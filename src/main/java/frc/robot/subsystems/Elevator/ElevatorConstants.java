package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorConstants {
    public static final double MAX_HEIGHT = 8840000;
    public static final double MAX_ANGLE = 884000000;
    public static final double MIN_ANGLE = -884000000;

    public static final double P_ELEVATOR = 0.217;
    public static final double I_ELEVATOR = 0.0;
    public static final double D_ELEVATOR = 0.0;

    public static final double P_ANGLE = 0.217;
    public static final double I_ANGLE = 0.0;
    public static final double D_ANGLE = 0.0;

    public static final double ROT_2_M = 0.0;
    public static final double ROT_2_RADIAN = 0.0;

    public static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS =
        new TrapezoidProfile.Constraints(10, 20);

    public static final TrapezoidProfile.Constraints HEIGHT_CONSTRAINTS =
        new TrapezoidProfile.Constraints(10, 20);
}

