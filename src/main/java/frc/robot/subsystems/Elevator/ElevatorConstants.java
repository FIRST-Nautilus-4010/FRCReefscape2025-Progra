package frc.robot.subsystems.Elevator;

/**
 * Contains all configuration constants for the Elevator subsystem,
 * including motion limits, PID gains, conversion factors, and profile constraints.
 */
public final class ElevatorConstants {
    // Prevent instantiation
    private ElevatorConstants() {}

    // --- Physical limits (units: meters) ---
    /** Maximum elevator height (meters) — TODO: replace placeholder. */
    public static final double MAX_HEIGHT = 8840000; // meters

    // --- Conversion Factors ---
    /** Conversion from motor rotations to meters. TODO: calibrate. */
    public static final double ROT_2_M = 0.0;

    // --- MM(Motion Magic) Configuration ---
    public static final double MAGIC_MOTION_VELOCITY = 0.0; // rot/s
    public static final double MAGIC_MOTION_ACCELERATION = 0.0; // rot/s²
    public static final double MAGIC_MOTION_JERK = 0.0; // rot/s³

    // --- MM Gains for Slot 0 ---
    /* Factors for MM TODO: calibrate (The output is in volts units)*/
    public static final double SLOT0_KG = 0.0; // output to overcome gravity (output)
    public static final double SLOT0_KS = 0.0; // output to overcome static friction (output)
    public static final double SLOT0_KV = 0.0; // output per unit of target velocity (output/rps)
    public static final double SLOT0_KA = 0.0; // output per unit of target acceleration (output/(rps/s))
    public static final double SLOT0_KP = 0.0; // output per unit of error in position (output/rotation)
    public static final double SLOT0_KI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
    public static final double SLOT0_KD = 0.0; // output per unit of error in velocity (output/rps)

    // --- Hardware IDs ---
    public static final int KRAKEN_RR_ID = 10; // Right rear motor ID
    public static final int KRAKEN_RL_ID = 11; // Right left motor ID
    public static final int KRAKEN_LR_ID = 12; // Left rear motor ID
    public static final int KRAKEN_LL_ID = 13; // Left left motor ID

    // --- Predefined Positions ---
    /* TODO: calibrate*/
    public static final double L1_POS = 0 / ROT_2_M;
    public static final double L2_POS = 0 / ROT_2_M;
    public static final double L3_POS = 0 / ROT_2_M;
    public static final double L4_POS = 0 / ROT_2_M;

    public static final double INTAKE_POS = 0 / ROT_2_M;

}
