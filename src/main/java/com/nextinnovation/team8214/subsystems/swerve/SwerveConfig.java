package com.nextinnovation.team8214.subsystems.swerve;

import com.nextinnovation.lib.geometry.Translation2d;

import java.util.Arrays;
import java.util.List;

public final class SwerveConfig {
  // Swerve
  public static final double WHEELBASE_HALF_LENGTH = 559.35 / 2.0 / 25.4;
  public static final double WHEELBASE_HALF_WIDTH = 559.35 / 2.0 / 25.4;
  public static final int MODULE_COUNT = 4;

  public static final Translation2d FRONT_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER =
      new Translation2d(WHEELBASE_HALF_LENGTH, WHEELBASE_HALF_WIDTH);
  public static final Translation2d REAR_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER =
      new Translation2d(-WHEELBASE_HALF_LENGTH, WHEELBASE_HALF_WIDTH);
  public static final Translation2d REAR_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER =
      new Translation2d(-WHEELBASE_HALF_LENGTH, -WHEELBASE_HALF_WIDTH);
  public static final Translation2d FRONT_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER =
      new Translation2d(WHEELBASE_HALF_LENGTH, -WHEELBASE_HALF_WIDTH);
  public static final List<Translation2d> POSITIONS_RELATIVE_TO_DRIVE_CENTER =
      Arrays.asList(
          FRONT_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER,
          REAR_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER,
          REAR_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER,
          FRONT_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER);

  // Calibration Offsets (calibration encoder values when the wheels are facing 0 degrees)
  public static final int FRONT_LEFT_CALIBRATION_OFFSET = 2460;
  public static final int REAR_LEFT_CALIBRATION_OFFSET = 5520;
  public static final int REAR_RIGHT_CALIBRATION_OFFSET = 12720;
  public static final int FRONT_RIGHT_CALIBRATION_OFFSET = 12528;

  // Odometer
  public static final double WHEEL_ODOMETER_MAX_DELTA_INCH = 2.2;

  // Speed Config
  public static final double MAX_SPEED_INCHES_PER_SECOND = 180.0;

  public static final class HeadingController {
    public static final class Manual {
      public static double KP = 0.375;
      public static double KI = 0.0;
      public static double KD = 0.01;
      public static double ERROR_TOLERANCE = 1.0 / 3.0;
    }

    public static final class Auto {
      public static double KP = 0.65;
      public static double KI = 0.0;
      public static double KD = 0.1;
      public static double ERROR_TOLERANCE = 1.0 / 5.0;
    }
  }
}
