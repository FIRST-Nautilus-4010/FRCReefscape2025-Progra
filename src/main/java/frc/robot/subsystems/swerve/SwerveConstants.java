package frc.robot.subsystems.swerve;

public class SwerveConstants {
    //--------Rev Robotics--------

    // SPARKS
    public static final int FL_STR = 1;
    public static final int FR_STR = 2;
    public static final int BL_STR = 3;
    public static final int BR_STR = 4;


    //--------CTR Electronics--------

    // Krakens 
    public static final int FL_PWR = 1;
    public static final int FR_PWR = 2;
    public static final int BL_PWR = 3;
    public static final int BR_PWR = 4;

    // Swerve encoders
    public static final int FL_ENC = 5;
    public static final int FR_ENC = 6;
    public static final int BL_ENC = 7;
    public static final int BR_ENC = 8;

    // Gyro
    public static final int PIGEON = 9;


    // Encoder offsets
    public static final double[] ENCODER_OFFSETS = {0, 0, 0, 0}; // <-- {FL, FR, BL, BR} offsets.

    // Wheel specifications
    public static final double WHEEL_DIAMETER = 0.1; // <-- In meters.

    // Motor ratios
    public static final double PWR_RATIO = 6.12; // <-- Power motor ratio.
    public static final double STR_RATIO = 12.8; // <-- Turning motor gear ratio.
    
    // Encoder conversions
    public static final double ROT_2_M = Math.PI * WHEEL_DIAMETER / PWR_RATIO; // <-- Converts motor rotations to meters.
    public static final double TURNING_ROT_2_RAD =  2 * Math.PI / STR_RATIO; // <-- Converts turning motor rotations to radians.
    public static final double TURNING_RPM_2_RAD_S = 5676 / 60.0 * TURNING_ROT_2_RAD;
    
    // PID constants
    public static final double PID_P = 0.097; // <-- Proportional gain.
    public static final double PID_I = 0.000000004010; // <-- Integral gain.
    public static final double PID_D = 0.000267; // <-- Derirvative gain. 
    
    // --- MM(Motion Magic) Configuration ---
    public static final double MAGIC_MOTION_VELOCITY = 95; // rot/s
    public static final double MAGIC_MOTION_ACCELERATION = 160; // rot/s²
    public static final double MAGIC_MOTION_JERK = 1600; // rot/s³
    public static final double MAGIC_MOTION_EXPO_KV = 0.12;
    public static final double MAGIC_MOTION_EXPO_KA = 0.1;
 
    /* Factors for MM TODO: calibrate (The output is in volts units)*/
    public static final double VEL_KG = 0.0; // output to overcome gravity (output)
    public static final double VEL_KS = 0.25; // output to overcome static friction (output)
    public static final double VEL_KV = 0.12; // output per unit of target velocity (output/rps)
    public static final double VEL_KA = 0.01; // output per unit of target acceleration (output/(rps/s))
    public static final double VEL_KP = 0.12; // output per unit of error in velocity (output/rps)
    public static final double VEL_KI = 0.0; // output per unit of integrated error in velocity (output/rotation)
    public static final double VEL_KD = 0.0; // output per unit of error derivate in velocity (output/rps/s)
}
