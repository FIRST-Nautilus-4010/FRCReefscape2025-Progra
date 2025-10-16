package frc.robot.subsystems.elevators;

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
    public static final double ROT_2_M = .46 / 15;

    // --- MM(Motion Magic) Configuration ---
    public static final double MAGIC_MOTION_VELOCITY = 95; // rot/s
    public static final double MAGIC_MOTION_ACCELERATION = 160; // rot/s²
    public static final double MAGIC_MOTION_JERK = 1600; // rot/s³
    public static final double MAGIC_MOTION_EXPO_KV = 0.12;
    public static final double MAGIC_MOTION_EXPO_KA = 0.1;


    // --- MM Gains for Slot 0 ---
    /* Factors for MM TODO: calibrate (The output is in volts units)*/
    public static final double POS_KG = 0.2; // output to overcome gravity (output)
    public static final double POS_KS = 0.25; // output to overcome static friction (output)
    public static final double POS_KV = 0.12; // output per unit of target velocity (output/rps)
    public static final double POS_KA = 0.01; // output per unit of target acceleration (output/(rps/s))
    public static final double POS_KP = 4.8; // output per unit of error in position (output/rotation)
    public static final double POS_KI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
    public static final double POS_KD = 0.1; // output per unit of error in velocity (output/rps)

    // --- Hardware IDs ---
    public static final int KRAKEN_RR_ID = 16; // Right rear motor ID
    public static final int KRAKEN_RL_ID = 17; // Right left motor ID
    public static final int KRAKEN_LR_ID = 22; // Left rear motor ID
    public static final int KRAKEN_LL_ID = 23; // Left left motor ID

    // --- Predefined Positions ---
    /* TODO: calibrate*/
    public static final double L1_POS = .3;
    public static final double L2_POS = .3;
    public static final double L3_POS = .30;
    public static final double L4_POS = .52;
    public static final double TRAVEL_POS = .3;
    public static final double PUT_L2L3_POS = .3;

    public static final double INTAKE_POS = .3;

}
