package frc.robot.subsystems.swerve;

public class SwerveConstants {
    //--------CTR Electronics--------

    // Krakens 
    public static final int FL_PWR = 4;
    public static final int FR_PWR = 3;
    public static final int BL_PWR = 2;
    public static final int BR_PWR = 1;

    public static final int FL_STR = 8;
    public static final int FR_STR = 7;
    public static final int BL_STR = 6;
    public static final int BR_STR = 5;

    // Swerve encoders
    public static final int FL_ENC = 12;
    public static final int FR_ENC = 11;
    public static final int BL_ENC = 10;
    public static final int BR_ENC = 9;

    // Gyro
    public static final int PIGEON = 13;

    // Wheel specifications. TODO: calibrate frictionCof
    public static final double WHEEL_DIAMETER = 0.1; // <-- In meters.
    public static final double MAX_HEIGHT = 2.2;
    private static final double FRICTION_COF = .95;

    // Motor ratios
    public static final double PWR_RATIO = 6.12; // <-- Power motor ratio.
    public static final double STR_RATIO = 12.8; // <-- Turning motor gear ratio.
    
    // Encoder conversions
    public static final double ROT_2_M = (Math.PI * WHEEL_DIAMETER) / PWR_RATIO; // <-- Converts motor rotations to meters.
    public static final double ROT_2_RAD =  (2 * Math.PI) / STR_RATIO; // <-- Converts turning motor rotations to radians.
    
    // --- MM(Motion Magic) Configuration ---
    public static final double MAGIC_MOTION_ACC = 400; // rot/s
    public static final double MAGIC_MOTION_JERK = 4000; // rot/s³

    // --- Max Motion Configuration ---
    public static final double MAX_MOTION_VEL = 5600; // rot/m
    public static final double MAX_MOTION_ACC = 50000; // rot/m²
    public static final double MAX_MOTION_ALLOWED_ERR = 1; // rot

    // --- Pos config ---

    // --- MM(Motion Magic) Configuration STR---
    public static final double MAGIC_MOTION_VELOCITY_STR = 95; // rot/s
    public static final double MAGIC_MOTION_ACCELERATION_STR = 160; // rot/s²
    public static final double MAGIC_MOTION_JERK_STR = 1600; // rot/s³
    public static final double MAGIC_MOTION_EXPO_KV_STR = 0.12;
    public static final double MAGIC_MOTION_EXPO_KA_STR = 0.1;


    // --- MM Gains for Slot 0 ---
    /* Factors for MM TODO: calibrate (The output is in volts units) STR*/
    public static final double POS_KG = 0.2; // output to overcome gravity (output)
    public static final double POS_KS = 0.25; // output to overcome static friction (output)
    public static final double POS_KV = 0.12; // output per unit of target velocity (output/rps)
    public static final double POS_KA = 0.01; // output per unit of target acceleration (output/(rps/s))
    public static final double POS_KP = 4.8; // output per unit of error in position (output/rotation)
    public static final double POS_KI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
    public static final double POS_KD = 0.1; // output per unit of error in velocity (output/rps)

    // --- Acceleration Limits ---
    public static final double MAX_FORDWARD_ACCEL = 10; // <-- in m/s.
    public static final double MAX_FRONT_ACCEL = 10; // <-- in m/s.
    public static final double MAX_SIDE_ACCEL = 10; // <-- in m/s.
    public static final double MAX_FRONT_ACCEL_MAX = 10;
    public static final double MAX_SIDE_ACCEL_MAX = 10;
    public static final double MAX_SKID_ACCEL = FRICTION_COF * 9.81; // <-- in m/s.
    //public static final double MAX_SKID_ACCEL = 15;
    public static final double VELOCITY_DEADZONE = .1; // zona muerta cerca de 0
 
    /* Factors for MM TODO: calibrate (The output is in volts units)*/
    public static final double VEL_KS = 0.10442; // output to overcome static friction (output)
    public static final double VEL_KV = 0.10882; // output per unit of target velocity (output/rps)
    public static final double VEL_KA = 0.001647; // output per unit of target acceleration (output/(rps/s))
    public static final double VEL_KP = 0.34; // output per unit of error in velocity (output/rps)
    public static final double VEL_KI = 0.00; // output per unit of integrated error in velocity (output/rotation)
    public static final double VEL_KD = 0.003; // output per unit of error derivate in velocity (output/rps/s)
}
