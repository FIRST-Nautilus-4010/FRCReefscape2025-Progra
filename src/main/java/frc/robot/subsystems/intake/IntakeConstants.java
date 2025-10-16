package frc.robot.subsystems.intake;

public class IntakeConstants {
    
    private IntakeConstants(){}

    public static final int motorPosition = 20;
    public static final int motorVelocity = 21;

    // --- MM Gains for Slot 0 ---
    /* Factors for MM TODO: calibrate (The output is in volts units)*/
    public static final double POS_KG = 0.2; // output to overcome gravity (output)
    public static final double POS_KS = 0.25; // output to overcome static friction (output)
    public static final double POS_KV = 0.12; // output per unit of target velocity (output/rps)
    public static final double POS_KA = 0.01; // output per unit of target acceleration (output/(rps/s))
    public static final double POS_KP = 4.8; // output per unit of error in position (output/rotation)
    public static final double POS_KI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
    public static final double POS_KD = 0.1; // output per unit of error in velocity (output/rps)

    /* Factors for MM TODO: calibrate (The output is in volts units)*/
    public static final double VEL_KS = 0.10442; // output to overcome static friction (output)
    public static final double VEL_KV = 0.10882; // output per unit of target velocity (output/rps)
    public static final double VEL_KA = 0.001647; // output per unit of target acceleration (output/(rps/s))
    public static final double VEL_KP = 0.34; // output per unit of error in velocity (output/rps)
    public static final double VEL_KI = 0.00; // output per unit of integrated error in velocity (output/rotation)
    public static final double VEL_KD = 0.003; // output per unit of error derivate in velocity (output/rps/s)

    // --- MM(Motion Magic) Configuration ---
    public static final double MAGIC_MOTION_ACC = 400; // rot/s
    public static final double MAGIC_MOTION_JERK = 4000; // rot/s³

    // --- MM(Motion Magic) Configuration STR---
    public static final double MAGIC_MOTION_VELOCITY_STR = 95; // rot/s
    public static final double MAGIC_MOTION_ACCELERATION_STR = 160; // rot/s²
    public static final double MAGIC_MOTION_JERK_STR = 1600; // rot/s³
    public static final double MAGIC_MOTION_EXPO_KV_STR = 0.12;
    public static final double MAGIC_MOTION_EXPO_KA_STR = 0.1;
}
