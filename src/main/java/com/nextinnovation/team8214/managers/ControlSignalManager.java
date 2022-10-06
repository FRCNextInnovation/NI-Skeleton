package com.nextinnovation.team8214.managers;

import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.io.StatefulXboxController;
import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.lib.utils.Util;
import com.nextinnovation.team8214.Ports;
import edu.wpi.first.wpilibj.DriverStation;

public class ControlSignalManager {
  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  public void registerEnabledLoops(ILooper enabledLooper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            resetControlFlags();
          }

          @Override
          public void onLoop(double timestamp) {
            if (!DriverStation.isAutonomousEnabled()) {
              update();
            }
          }

          @Override
          public void onStop(double timestamp) {
            resetControlFlags();
          }
        };
    enabledLooper.register(loop);
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static ControlSignalManager instance = null;

  public static synchronized ControlSignalManager getInstance() {
    if (instance == null) {
      instance = new ControlSignalManager();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final StatefulXboxController driverController;

  private final StatefulXboxController codriverController;

  private ControlSignalManager() {
    driverController =
        new StatefulXboxController(Ports.DriverJoysticks.DRIVER_CONTROLLER_PORT, 0.5);
    codriverController =
        new StatefulXboxController(Ports.DriverJoysticks.CODRIVER_CONTROLLER_PORT, 0.5);

    resetControlFlags();
  }

  /************************************************************************************************
   * Reset *
   ************************************************************************************************/
  public synchronized void resetControlFlags() {}

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public Translation2d getSwerveManualTranslation() {
    Translation2d input =
        Util.applyRemappedCircularDeadband(
                new Translation2d(-driverController.getRightY(), -driverController.getRightX()),
                0.09375)
            .scale(0.75);

    return input;
  }

  public double getSwerveManualRotationMagnitude() {
    double input = Util.applyRemappedDeadband(-driverController.getLeftX(), 0.09375) * 0.27;

    return input;
  }

  /************************************************************************************************
   * Update *
   ************************************************************************************************/
  private synchronized void update() {
    // Use when in manual mode
    driverController.updateButtons();
    codriverController.updateButtons();
  }

  /************************************************************************************************
   * Log *
   ************************************************************************************************/
  public void logToSmartDashBoard() {}
}
