package com.nextinnovation.team8214.managers;

import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.io.StatefulXboxController;
import com.nextinnovation.lib.utils.Util;
import com.nextinnovation.team8214.Ports;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class ControlSignalManager {
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

  private final SlewRateLimiter swerveTranslationFilter = new SlewRateLimiter(1.0);
  private final SlewRateLimiter swerveRotationFilter = new SlewRateLimiter(1.0);

  private ControlSignalManager() {
    driverController =
        new StatefulXboxController(Ports.DriverJoysticks.DRIVER_CONTROLLER_PORT, 0.5);
    codriverController =
        new StatefulXboxController(Ports.DriverJoysticks.CODRIVER_CONTROLLER_PORT, 0.5);
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public Translation2d getSwerveManualTranslation() {
    Translation2d input =
        Util.applyRemappedCircularDeadband(
                new Translation2d(-driverController.getRightY(), -driverController.getRightX()),
                0.09375)
            .scale(0.6);
    if (driverController.getButton(StatefulXboxController.ButtonId.TRIGGER_LEFT).isBeingPressed()) {
      input = input.scale(0.5);
    }

    input =
        Translation2d.fromPolar(
            input.direction(), swerveTranslationFilter.calculate(Math.pow(input.norm(), 2)));

    return input;
  }

  public double getSwerveManualRotationMagnitude() {
    double input = Util.applyRemappedDeadband(-driverController.getLeftX(), 0.09375) * 0.5;

    if (driverController.getButton(StatefulXboxController.ButtonId.TRIGGER_LEFT).isBeingPressed()) {
      input *= (0.5);
    }

    return swerveRotationFilter.calculate(input);
  }

  /************************************************************************************************
   * Update *
   ************************************************************************************************/
  public synchronized void update() {
    driverController.updateButtons();
    codriverController.updateButtons();
  }
}
