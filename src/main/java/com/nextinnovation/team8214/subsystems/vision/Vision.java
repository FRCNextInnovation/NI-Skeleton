package com.nextinnovation.team8214.subsystems.vision;

import com.nextinnovation.lib.drivers.Limelight;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Field;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends BaseSubsystem {
  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            setState(VisionState.ENABLE);
          }

          @Override
          public void onLoop(double timestamp) {}

          @Override
          public void onStop(double timestamp) {
            disable();
          }
        };
    enabledLooper.register(loop);
  }

  /***********************************************************************************************
   * Periodic IO *
   ***********************************************************************************************/
  private static class PeriodicInput {
    double latency = 255.0;
    boolean hasTarget = false;
    Rotation2d targetX = Rotation2d.identity();
    Rotation2d targetY = Rotation2d.identity();
  }

  @Override
  public void readPeriodicInputs() {
    synchronized (ioLock) {
      periodicInput.latency = limelight.getLatency();
      periodicInput.hasTarget = limelight.hasTarget();
      if (periodicInput.hasTarget) {
        periodicInput.targetX = limelight.getTargetX();
        periodicInput.targetY = limelight.getTargetY();
      }
    }
  }
  /***********************************************************************************************
   * Subsystem States *
   ***********************************************************************************************/
  private VisionState visionState = VisionState.OFF;

  public synchronized void setState(VisionState new_state) {
    visionState = new_state;
    switch (visionState) {
      case OFF:
        off();
        break;
      case ENABLE:
        enable();
        break;
      case BLINK:
        blink();
        break;
    }
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static Vision instance = null;

  public static synchronized Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final PeriodicInput periodicInput = new PeriodicInput();

  private final Limelight limelight = new Limelight();
  private boolean isEnabled = false;

  private final Object ioLock = new Object();

  public Vision() {
    setState(VisionState.OFF);
  }

  /************************************************************************************************
   * Function Enabler *
   ************************************************************************************************/
  private void enable() {
    isEnabled = true;
    limelight.setLedMode(0);
  }

  private void off() {
    isEnabled = false;
    limelight.setLedMode(1);
  }

  private void blink() {
    isEnabled = false;
    limelight.setLedMode(2);
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public boolean isEnabled() {
    return isEnabled;
  }

  public boolean hasTarget() {
    synchronized (ioLock) {
      return periodicInput.hasTarget && isEnabled;
    }
  }

  public Rotation2d getSceneCentricTargetAngle() {
    synchronized (ioLock) {
      return periodicInput.targetY;
    }
  }

  public Rotation2d getTargetHeading() {
    synchronized (ioLock) {
      return periodicInput.targetX.inverse();
    }
  }

  /**
   * Get camera latency in ms
   *
   * @return camera latency in ms
   */
  public double getLatency() {
    synchronized (ioLock) {
      return periodicInput.latency / 1000.0;
    }
  }

  /**
   * Get camera centric distance from vision target in inch
   *
   * @return target distance in inch
   */
  public double getTargetDistance() {
    return (Field.VISUAL_TARGET_VISUAL_CENTER_HEIGHT - VisionConfig.CAMERA_HEIGHT_INCH)
        / Math.tan(
            getSceneCentricTargetAngle().getRadians() + VisionConfig.CAMERA_ELEVATION.getRadians());
  }

  /**
   * Get camera centric orientation from vision target with distance in inch
   *
   * @return target distance in inch by a Translation2d
   */
  public Translation2d getTargetOrientation() {
    return Translation2d.fromPolar(getTargetHeading(), getTargetDistance());
  }

  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  @Override
  public void disable() {
    setState(VisionState.OFF);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  @Override
  public void logToSmartDashboard() {
    SmartDashboard.putString("Vision State", visionState.value);
    SmartDashboard.putBoolean("Has Vision Target", hasTarget());
    SmartDashboard.putBoolean("Is Vision Enabled", isEnabled());
    if (Config.ENABLE_DEBUG_OUTPUT) {
      SmartDashboard.putNumber("Camera Latency", getLatency());
      if (isEnabled() && hasTarget()) {
        SmartDashboard.putString("Target Orientation", getTargetOrientation().toString());
      }
    }
  }
}
