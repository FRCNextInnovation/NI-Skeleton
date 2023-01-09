package com.nextinnovation.team8214.managers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.*;
import com.nextinnovation.lib.drivers.CanId;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Ports;
import edu.wpi.first.wpilibj.Timer;

public class CancoderManager {
  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static CancoderManager instance = null;

  public static synchronized CancoderManager getInstance() {
    if (instance == null) {
      instance = new CancoderManager();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private static final double MAX_INIT_TIME = 10.0;

  private final CANCoder frontLeft;
  private final CANCoder rearLeft;
  private final CANCoder rearRight;
  private final CANCoder frontRight;

  private CancoderManager() {
    frontLeft = build(Ports.Can.FRONT_LEFT_ROTATION_SENSOR);
    rearLeft = build(Ports.Can.REAR_LEFT_ROTATION_SENSOR);
    rearRight = build(Ports.Can.REAR_RIGHT_ROTATION_SENSOR);
    frontRight = build(Ports.Can.FRONT_RIGHT_ROTATION_SENSOR);
  }

  private CANCoder build(CanId id) {
    var canCoder = new CANCoder(id.getNum(), id.getBus());
    var config = new CANCoderConfiguration();

    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = 0.0;
    config.sensorDirection = false; // Counter-clockwise

    double startTime = Timer.getFPGATimestamp();
    boolean isTimeOut = false;
    boolean isInitDone = false;

    while (!isTimeOut && !isInitDone) {
      var initError = canCoder.configAllSettings(config, Config.CAN_TIMEOUT_MS);
      var statusFrameError =
          canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 50, Config.CAN_TIMEOUT_MS);

      isInitDone = (initError == ErrorCode.OK && statusFrameError == ErrorCode.OK);
      isTimeOut = Timer.getFPGATimestamp() - startTime >= MAX_INIT_TIME;
    }

    return canCoder;
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public CANCoder getFrontLeft() {
    return frontLeft;
  }

  public CANCoder getRearLeft() {
    return rearLeft;
  }

  public CANCoder getRearRight() {
    return rearRight;
  }

  public CANCoder getFrontRight() {
    return frontRight;
  }
}
