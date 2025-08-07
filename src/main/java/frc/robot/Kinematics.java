package frc.robot;

public class Kinematics {

    /**
     * This code calculates the angles of the two joints (theta1 and theta2)
     * given the end effector position (x, y) and the angle of the end effector (theta).
     * The arm is assumed to have two segments of lengths a and b.
     * @param x The x-coordinate of the end effector
     * @param y The y-coordinate of the end effector
     * @param theta The angle of the end effector in radians
     * @return An array containing the angles theta1, theta2, and the height of the end effector
     *         theta1 is the angle of the first joint, theta2 is the angle of the second joint,
     * *         and height is the vertical position of the end effector.
     *         The angles are in radians and the height is in meters.
     */
    public static double[] inverseKinematics(double x, double y, double theta) {
        double theta1 = Math.asin((-x - (0.19345982 * Math.sin(theta))) / 0.28597);
        double theta2 = theta - theta1;
        double height = y - (0.28597 * Math.cos(theta1) + 0.19345982 * Math.cos(theta));
        
        return new double[] {theta1, theta2, height};
    }
}
