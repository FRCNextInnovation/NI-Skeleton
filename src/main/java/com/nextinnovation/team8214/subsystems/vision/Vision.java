package com.nextinnovation.team8214.subsystems.vision;

import com.nextinnovation.lib.drivers.Limelight;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import com.nextinnovation.lib.utils.InterpolatingDouble;
import com.nextinnovation.lib.utils.Units;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Field;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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
          public void onLoop(double timestamp) {
            readPeriodicInputs();
          }

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
  public static class VisionTargetInfo {
    public Rotation2d targetHeading;
    public Rotation2d targetElevation;
    public double targetDistance;
    public double timestamp;

    public VisionTargetInfo(
        Rotation2d target_heading,
        Rotation2d target_elevation,
        double target_distance,
        double timestamp) {
      targetHeading = target_heading;
      targetElevation = target_elevation;
      targetDistance = target_distance;
      this.timestamp = timestamp;
    }
  }

  private static class PeriodicInput {
    double latency = 255.0;
    double timestamp = 0.0;
    boolean isUpdated = false;
    boolean hasTarget = false;
    Rotation2d targetX = Rotation2d.identity();
    Rotation2d targetY = Rotation2d.identity();
  }

  @Override
  public void readPeriodicInputs() {
    synchronized (ioLock) {
      periodicInput.isUpdated = true;
      periodicInput.latency = limelight.getLatency();
      periodicInput.timestamp = Timer.getFPGATimestamp() - periodicInput.latency;
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
    configSmartDashboard();
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

  public boolean isUpdated() {
    synchronized (ioLock) {
      if (periodicInput.isUpdated) {
        periodicInput.isUpdated = false;
        return true;
      } else {
        return false;
      }
    }
  }

  public boolean hasTarget() {
    synchronized (ioLock) {
      return periodicInput.hasTarget && isEnabled;
    }
  }

  private Rotation2d getTargetElevation() {
    synchronized (ioLock) {
      return periodicInput.targetY;
    }
  }

  private Rotation2d getTargetHeading() {
    synchronized (ioLock) {
      return periodicInput.targetX.inverse();
    }
  }

  /**
   * Get camera latency in ms
   *
   * @return camera latency in ms
   */
  private double getLatency() {
    synchronized (ioLock) {
      return periodicInput.latency / 1000.0;
    }
  }

  /**
   * Get target timestamp in seconds
   *
   * @return target timestamp in seconds
   */
  private double getTargetTimestamp() {
    synchronized (ioLock) {
      return periodicInput.timestamp;
    }
  }

  /**
   * Get camera centric distance from vision target in inch
   *
   * @return target distance in inch
   */
  private double getTargetDistance() {
    double realDistanceInch =
        (Field.VISUAL_TARGET_VISUAL_CENTER_HEIGHT - VisionConfig.CAMERA_HEIGHT_INCH)
            / Math.tan(
                getTargetElevation().getRadians() + VisionConfig.CAMERA_ELEVATION.getRadians());

    return VisionConfig.visionDistanceRemappedMap.getInterpolated(
            new InterpolatingDouble(realDistanceInch))
        .value;
  }

  /**
   * Get camera centric orientation from vision target with distance in inch
   *
   * @return target distance in inch by a Translation2d
   */
  public Translation2d getTargetOrientation() {
    return Translation2d.fromPolar(getTargetHeading(), getTargetDistance());
  }

  /**
   * Get camera centric vision target info
   *
   * @return vision target info
   */
  public VisionTargetInfo getVisionTargetInfo() {
    synchronized (ioLock) {
      return new VisionTargetInfo(
          getTargetHeading(), getTargetElevation(), getTargetDistance(), getTargetTimestamp());
    }
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
  private ShuffleboardTab tab;

  private NetworkTableEntry visionStateEntry;
  private NetworkTableEntry hasVisionTargetEntry;
  private NetworkTableEntry targetOrientationEntry;
  private NetworkTableEntry targetElevationEntry;
  private NetworkTableEntry fixedDistanceEntry;
  private NetworkTableEntry cameraLatencyEntry;

  public void configSmartDashboard() {
    tab = Shuffleboard.getTab("Vision");

    visionStateEntry = tab.add("Vision State", "None").getEntry();
    hasVisionTargetEntry = tab.add("Has Vision Target", false).getEntry();
    targetOrientationEntry = tab.add("Target Orientation", 0.0).getEntry();
    targetElevationEntry = tab.add("Target Elevation", 0.0).getEntry();
    fixedDistanceEntry = tab.add("Fixed Distance", 99.99).getEntry();
    cameraLatencyEntry = tab.add("Camera Latency", 99.99).getEntry();
  }

  @Override
  public void logToSmartDashboard() {
    if (Config.ENABLE_DEBUG_OUTPUT) {
      visionStateEntry.setString(visionState.value);
      hasVisionTargetEntry.setBoolean(hasTarget());
      targetOrientationEntry.setNumber(getTargetOrientation().direction().getDegrees());
      targetElevationEntry.setNumber(getTargetElevation().getDegrees());
      fixedDistanceEntry.setNumber(Units.inches_to_meters(getTargetDistance()));
      cameraLatencyEntry.setNumber(getLatency());
    }
  }
}
