package com.nextinnovation.team8214.devices.ahrs;

import com.nextinnovation.lib.drivers.NavX;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.team8214.Config;
import edu.wpi.first.wpilibj.Timer;

public class AhrsNavX extends BaseAhrs {
  private static AhrsNavX instance = null;

  public static synchronized BaseAhrs getInstance() {
    if (instance == null) {
      instance = new AhrsNavX();
    }
    return instance;
  }

  private static final NavX navx = new NavX();
  private static Rotation2d referenceHeading = Rotation2d.identity();

  private AhrsNavX() {
    init();
  }

  private void init() {
    var initTime = Timer.getFPGATimestamp();
    while (!navx.isConnected()) {
      if (Timer.getFPGATimestamp() - initTime > 4.0) {
        System.out.print("Warning: NavX initialization timed out with ");
        System.out.print(Timer.getFPGATimestamp() - initTime);
        System.out.println(" seconds");
        break;
      }
      Timer.delay(0.01);
    }
    setRobotHeading(Rotation2d.fromDegrees(Config.INIT_HEADING));
    Timer.delay(0.5);
  }

  @Override
  public boolean isReady() {
    return navx.isConnected();
  }

  @Override
  public Rotation2d getRobotHeading() {
    return Rotation2d.fromDegrees(navx.getFusedYaw()).rotateBy(referenceHeading.inverse());
  }

  @Override
  public Rotation2d getRobotAngularVelocity() {
    return Rotation2d.fromDegrees(navx.getGyroZ());
  }

  @Override
  public synchronized void setRobotHeading(Rotation2d heading) {
    referenceHeading = getRobotHeading().rotateBy(heading.inverse());
  }

  @Override
  public synchronized void resetRobotHeading() {
    setRobotHeading(Rotation2d.identity());
  }
}
