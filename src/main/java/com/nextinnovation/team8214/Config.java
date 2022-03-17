package com.nextinnovation.team8214;

import com.nextinnovation.lib.utils.Util;

/** Final class to store all constants except ID. */
public final class Config {
  // Robot Physical Dimensions (including bumpers)
  public static final double ROBOT_WIDTH = 35.98;
  public static final double ROBOT_LENGTH = 35.51;
  public static final double ROBOT_HALF_WIDTH = ROBOT_WIDTH / 2.0;
  public static final double ROBOT_HALF_LENGTH = ROBOT_LENGTH / 2.0;
  public static final double INTAKE_PUSH_OUT_LENGTH = 4.0;

  // AHRS
  public static final double INIT_HEADING = 180.0;

  // Lopper
  public static final double LOOPER_CONTROL_PERIOD_SEC = 0.01;
  public static final int LOOPER_CONTROL_DELTA_TIME_MS =
      Util.roundToInt(LOOPER_CONTROL_PERIOD_SEC * 1000.0);
  public static final double LOOPER_LOG_PERIOD_SEC = 0.04;

  // CAN
  public static final int CAN_TIMEOUT_MS = 100;
  public static final int CAN_INSTANT_TIMEOUT_MS = 30;

  // Log
  public static final boolean ENABLE_DEBUG_OUTPUT = true;
}
