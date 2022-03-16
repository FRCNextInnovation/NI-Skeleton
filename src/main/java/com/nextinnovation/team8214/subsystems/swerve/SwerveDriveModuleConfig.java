package com.nextinnovation.team8214.subsystems.swerve;

import com.nextinnovation.team8214.Config;

public final class SwerveDriveModuleConfig {
  // Scrub Factor
  public static final boolean ENABLE_SCRUB_FACTOR = true;
  public static final double X_SCRUB_FACTOR = 1.0 / (1.0 - (9549.0 / 293093.0));
  public static final double Y_SCRUB_FACTOR = 1.0 / (1.0 - (4.4736 / 119.9336));

  public static final class Translation {
    // Control Inversion
    public static final boolean IS_INVERT = true;

    // Speed Config
    public static final double MAX_SPEED = 18600.0;
    public static final double SPEED_RESERVE_FACTOR = 1 / 16.0;
    public static final double MAX_CRUISE_SPEED =
        MAX_SPEED * (1.0 - SPEED_RESERVE_FACTOR); // in encoder units / 0.1s
    public static final double OPEN_LOOP_RAMP = 0.12;
    public static final double CLOSED_LOOP_RAMP = 0.12;

    // Error Tolerance
    public static final double SPEED_ON_TARGET_ERROR_TOLERANCE = 1.0;
    public static final double DISTANCE_ON_TARGET_ERROR_TOLERANCE = 1.0;

    // PID Parameters
    // Slot 0 is for teleop
    public static final double PID0_KP = 0.0625;
    public static final double PID0_KI = 0.0;
    public static final double PID0_KD = 0.01;
    public static final double PID0_KF = 1023.0 / MAX_SPEED;

    // Communication Parameters
    public static final int STATUS_FRAME_PERIOD = Config.LOOPER_CONTROL_DELTA_TIME_MS;

    // Mechanical Config
    public static final double WHEEL_DIAMETER = 4.012;
    public static final int ENCODER_RESOLUTION = 2048;
    public static final double ENCODER_TO_WHEEL_RATIO =
        570.0 / 91.0; // the number of rotations the encoder undergoes for every rotation of the
    // wheel
    public static final double ENCODER_UNITS_PER_WHEEL_RESOLUTION =
        ENCODER_RESOLUTION * ENCODER_TO_WHEEL_RATIO;
    public static final double ODOMETER_ERROR_COMPENSATION_FACTOR = 1.0;
    public static final double ENCODER_UNITS_PER_INCH =
        ENCODER_UNITS_PER_WHEEL_RESOLUTION
            / (Math.PI * WHEEL_DIAMETER)
            * ODOMETER_ERROR_COMPENSATION_FACTOR;
  }

  public static final class Rotation {
    // Control Inversion
    public static final boolean IS_INVERT = true;
    public static final boolean IS_EXTERNAL_SENSOR_INVERT = true;

    // Speed Config
    public static final double SPEED_RESERVE_FACTOR = 1.0 / 16.0;
    public static final double MAX_SPEED = 14600.0; // in encoder units / 0.1s
    public static final double MAX_CRUISE_SPEED =
        MAX_SPEED * (1.0 - SPEED_RESERVE_FACTOR); // in encoder units / 0.1s
    public static final double MAX_ACCELERATION =
        MAX_SPEED * 16.0; // estimated, in encoder units / 0.1s^2

    // Error Tolerance
    public static final double HEADING_ON_TARGET_ERROR_TOLERANCE = 360.0 / 256.0;

    // PID Parameters
    // Slot 0 is for teleop
    public static final double PID0_KP = 2.0;
    public static final double PID0_KI = 0.0;
    public static final double PID0_KD = 24.0;
    public static final double PID0_KF = 1023.0 / MAX_SPEED;

    // Mechanical Config
    public static final int ENCODER_RESOLUTION = 2048;
    public static final int CALIBRATION_ENCODER_RESOLUTION = 4096;
    public static final double ENCODER_TO_MODULE_BASE_RATIO =
        12.0; // the number of rotations the encoder undergoes for every rotation of the base of
    // the module
    public static final double ENCODER_TO_EXTERNAL_ENCODER_RATIO =
        (double) (CALIBRATION_ENCODER_RESOLUTION)
            / ENCODER_RESOLUTION
            / ENCODER_TO_MODULE_BASE_RATIO; // the number of encoder units the encoder undergoes
    // for every encoder unit the calibration encoder
    // undergoes
    public static final double ENCODER_UNITS_PER_MODULE_BASE_REVOLUTION =
        ENCODER_RESOLUTION * ENCODER_TO_MODULE_BASE_RATIO;
    public static final double ENCODER_UNITS_PER_DEGREE =
        ENCODER_UNITS_PER_MODULE_BASE_REVOLUTION / 360.0;

    // Communication Parameters
    public static final int STATUS_FRAME_PERIOD = Config.LOOPER_CONTROL_DELTA_TIME_MS;
  }
}
