package frc.robot.subsystems.Elevator;

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

    // --- Conversion Factors ---
    /** Conversion from motor rotations to meters. TODO: calibrate. */
    public static final double ROT_2_M = 0.0;

    /** Conversion from motor rotations to radians. TODO: calibrate. */
    public static final double ROT_2_RADIAN = 0.0;

    // --- MM(Magic Motion) Configuration ---
    public static final double MAGIC_MOTION_VELOCITY = 0.0; // rot/s
    public static final double MAGIC_MOTION_ACCELERATION = 0.0; // rot/s²
    public static final double MAGIC_MOTION_JERK = 0.0; // rot/s³

    // --- MM Gains for Slot 0 (Angle Control) ---
    /* Every unit is in volts */
    public static final double SLOT0_KG = 0.0; // output to overcome gravity (output)
    public static final double SLOT0_KS = 0.0; // output to overcome static friction (output)
    public static final double SLOT0_KV = 0.0; // output per unit of target velocity (output/rps)
    public static final double SLOT0_KA = 0.0; // output per unit of target acceleration (output/(rps/s))
    public static final double SLOT0_KP = 0.0; // output per unit of error in position (output/rotation)
    public static final double SLOT0_KI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
    public static final double SLOT0_KD = 0.0; // output per unit of error in velocity (output/rps)

    // --- MM Gains for Slot 1 (Height Control) ---
    public static final double SLOT1_KG = 0.0; 
    public static final double SLOT1_KS = 0.0; 
    public static final double SLOT1_KV = 0.0; 
    public static final double SLOT1_KA = 0.0; 
    public static final double SLOT1_KP = 0.0; 
    public static final double SLOT1_KI = 0.0; 
    public static final double SLOT1_KD = 0.0; 

    // --- Hardware IDs ---
    public static final int KRAKEN_RR_ID = 10; // Right rear motor ID
    public static final int KRAKEN_RL_ID = 11; // Right left motor ID
    public static final int KRAKEN_LR_ID = 12; // Left rear motor ID
    public static final int KRAKEN_LL_ID = 13; // Left left motor ID
}
