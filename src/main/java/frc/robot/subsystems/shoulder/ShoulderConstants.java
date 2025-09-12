package frc.robot.subsystems.shoulder;


/**
 * Contains all configuration constants for the Elevator subsystem,
 * including motion limits, PID gains, conversion factors, and profile constraints.
 */
public class ShoulderConstants {
    // Prevent instantiation
    private ShoulderConstants() {}

    // --- Physical limits (units: meters) ---
    /** Maximum elevator height (meters) — TODO: replace placeholder. */
    public static final double MAX_ANGLE = 8840000; // meters
    public static final double MIN_ANGLE = -8840000;
    public static final double SAFETY_HEIGHT = .2;

    // --- Conversion Factors ---
    /** Conversion from motor rotations to meters. TODO: calibrate. */  
    public static final double ROT_2_RADIAN = (2 / 3) * Math.PI;

    // --- MM(Motion Magic) Configuration ---
    public static final double MAGIC_MOTION_VELOCITY = 95; // rot/s
    public static final double MAGIC_MOTION_ACCELERATION = 160; // rot/s²
    public static final double MAGIC_MOTION_JERK = 1600; // rot/s³
    public static final double MAGIC_MOTION_EXPO_KV = 0.12;
    public static final double MAGIC_MOTION_EXPO_KA = 0.1;
 
 
    // --- MM Gains for Slot 0 ---
    /* Factors for MM TODO: calibrate (The output is in volts units)*/
    public static final double POS_KG = 0.0; // output to overcome gravity (output)
    public static final double POS_KS = 0.25; // output to overcome static friction (output)
    public static final double POS_KV = 0.12; // output per unit of target velocity (output/rps)
    public static final double POS_KA = 0.01; // output per unit of target acceleration (output/(rps/s))
    public static final double POS_KP = 4.8; // output per unit of error in position (output/rotation)
    public static final double POS_KI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
    public static final double POS_KD = 0.1; // output per unit of error in velocity (output/rps)
 
    /* Factors for MM TODO: calibrate (The output is in volts units)*/
    public static final double VEL_KG = 0.0; // output to overcome gravity (output)
    public static final double VEL_KS = 0.25; // output to overcome static friction (output)
    public static final double VEL_KV = 0.12; // output per unit of target velocity (output/rps)
    public static final double VEL_KA = 0.01; // output per unit of target acceleration (output/(rps/s))
    public static final double VEL_KP = 0.12; // output per unit of error in velocity (output/rps)
    public static final double VEL_KI = 0.0; // output per unit of integrated error in velocity (output/rotation)
    public static final double VEL_KD = 0.0; // output per unit of error derivate in velocity (output/rps/s)

    // --- Hardware IDs ---
    public static final int KRAKEN_R_ID = 14; // Right rear motor ID
    public static final int KRAKEN_L_ID = 15; // Left motor ID

    // --- Predefined Positions ---
    /* TODO: calibrate*/
    public static final double L1_POS = 0 / ROT_2_RADIAN;
    public static final double L2_POS = (Math.PI) / ROT_2_RADIAN;
    public static final double L3_POS = 0 / ROT_2_RADIAN;
    public static final double L4_POS = 0 / ROT_2_RADIAN;

    public static final double INTAKE_POS = 0 / ROT_2_RADIAN;

    public static final double PUT = 1;
}
