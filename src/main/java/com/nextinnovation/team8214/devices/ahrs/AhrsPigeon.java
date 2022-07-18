package com.nextinnovation.team8214.devices.ahrs;

import com.nextinnovation.lib.drivers.Pigeon;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Ports;
import edu.wpi.first.wpilibj.Timer;

public class AhrsPigeon extends BaseAhrs {
  private static AhrsPigeon instance = null;

  public static synchronized BaseAhrs getInstance() {
    if (instance == null) {
      instance = new AhrsPigeon();
    }
    return instance;
  }

  private static final Pigeon pigeon = new Pigeon(Ports.Can.PIGEON_CHASSIS, Config.CAN_TIMEOUT_MS);
  private static Rotation2d referenceHeading = Rotation2d.identity();

  private AhrsPigeon() {
    init();
  }

  private void init() {
    double initTime = Timer.getFPGATimestamp();
    while (!pigeon.isReady()) {
      if (Timer.getFPGATimestamp() - initTime > 4.0) {
        System.out.print("Warning: Pigeon initialization timed out with ");
        System.out.print(Timer.getFPGATimestamp() - initTime);
        System.out.println(" seconds");
        break;
      }
      Timer.delay(0.01);
    }
    pigeon.configStatusFramePeriod(
        Config.LOOPER_CONTROL_DELTA_TIME_MS, false, Config.CAN_TIMEOUT_MS);
    pigeon.configTemperatureCompensation(true);
    setRobotHeading(Rotation2d.fromDegrees(Config.INIT_HEADING));
    Timer.delay(3.0);
  }

  @Override
  public boolean isReady() {
    return pigeon.isReady();
  }

  @Override
  public Rotation2d getRobotHeading() {
    return Rotation2d.fromDegrees(pigeon.getFusedYaw()).rotateBy(referenceHeading.inverse());
  }

  @Override
  public Rotation2d getRobotAngularVelocity() {
    return Rotation2d.fromDegrees(pigeon.getGyroZ());
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
