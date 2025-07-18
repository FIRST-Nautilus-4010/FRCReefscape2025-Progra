package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Contains all configuration constants for the Elevator subsystem,
 * including motion limits, PID gains, conversion factors, and profile constraints.
 */
public final class ElevatorConstants {
    // Prevent instantiation
    private ElevatorConstants() {}

    // --- Physical limits (units: meters and radians) ---
    /** Maximum elevator height (meters) — TODO: replace placeholder. */
    public static final double MAX_HEIGHT = 8840000; // meters

    /** Maximum rotation angle of the elevator arm (radians). */
    public static final double MAX_ANGLE = 884000000; // radians

    /** Minimum rotation angle of the elevator arm (radians). */
    public static final double MIN_ANGLE = -884000000; // radians

    // --- PID Gains for Height Control ---
    public static final double P_ELEVATOR = 0.217;
    public static final double I_ELEVATOR = 0.0;
    public static final double D_ELEVATOR = 0.0;

    // --- PID Gains for Angle Control ---
    public static final double P_ANGLE = 0.217;
    public static final double I_ANGLE = 0.0;
    public static final double D_ANGLE = 0.0;

    // --- Conversion Factors ---
    /** Conversion from motor rotations to meters. TODO: calibrate. */
    public static final double ROT_2_M = 0.0;

    /** Conversion from motor rotations to radians. TODO: calibrate. */
    public static final double ROT_2_RADIAN = 0.0;

    // --- Motion Profile Constraints ---
    /** Trapezoidal profile constraints for angle control (rad/s, rad/s²). */
    public static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS =
        new TrapezoidProfile.Constraints(10.0, 20.0);

    /** Trapezoidal profile constraints for height control (m/s, m/s²). */
    public static final TrapezoidProfile.Constraints HEIGHT_CONSTRAINTS =
        new TrapezoidProfile.Constraints(10.0, 20.0);
}
