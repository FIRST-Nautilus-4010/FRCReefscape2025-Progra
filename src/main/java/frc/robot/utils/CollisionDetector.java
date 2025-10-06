package frc.robot.utils;

import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.swerve.Swerve;

public class CollisionDetector {
  private static final double IMPACT_THRESHOLD = ChassisConstants.MAX_ACCEL + 1; // m/s²
  private boolean recentlyHit = false;
  private int freezeCounter = 0;

  public boolean detectImpact(Swerve swerve) {
    double accel = swerve.getLinearAcceleration(); // necesitas implementar en Swerve
    if (Math.abs(accel) > IMPACT_THRESHOLD) {
      recentlyHit = true;
      freezeCounter = 5; // congela 5 ciclos (~100 ms)
    }
    return recentlyHit;
  }

  public void freeze() {
    if (freezeCounter > 0) {
      freezeCounter--;
    } else {
      recentlyHit = false;
    }
  }
}
