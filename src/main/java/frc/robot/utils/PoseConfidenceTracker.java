package frc.robot.utils;

import frc.robot.subsystems.swerve.Swerve;

public class PoseConfidenceTracker {
  private boolean skidding = false;

  public void update(Swerve swerve) {
    double avgWheelSpeed = swerve.getAverageWheelSpeed();
    double chassisSpeed = swerve.getChassisSpeed();

    skidding = Math.abs(avgWheelSpeed - chassisSpeed) > 0.5; // ajusta umbral
  }

  public boolean isSkidding() {
    return skidding;
  }

  public boolean shouldTrustVision(edu.wpi.first.math.geometry.Pose2d vision, edu.wpi.first.math.geometry.Pose2d odom) {
    // No confiar si hay diferencia > 1 m entre visión y odometría
    return vision.getTranslation().getDistance(odom.getTranslation()) < 1.0;
  }
}
