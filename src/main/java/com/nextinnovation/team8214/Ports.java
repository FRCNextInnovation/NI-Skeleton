package com.nextinnovation.team8214;

/** Final class to store all ID of CAN devices or the solenoids connected to PCM. */
public final class Ports {
  /** Inner class to store IDs of CAN devices. */
  public static final class Can {
    public static final int PCM = 0;
    public static final int PIGEON_CHASSIS = 0;

    // Swerve Drive
    public static final int FRONT_LEFT_DRIVE_MOTOR = 0;
    public static final int FRONT_LEFT_ROTATION_MOTOR = 1;
    public static final int FRONT_LEFT_ROTATION_SENSOR = 8;

    public static final int REAR_LEFT_DRIVE_MOTOR = 2;
    public static final int REAR_LEFT_ROTATION_MOTOR = 3;
    public static final int REAR_LEFT_ROTATION_SENSOR = 9;

    public static final int REAR_RIGHT_DRIVE_MOTOR = 4;
    public static final int REAR_RIGHT_ROTATION_MOTOR = 5;
    public static final int REAR_RIGHT_ROTATION_SENSOR = 10;

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 6;
    public static final int FRONT_RIGHT_ROTATION_MOTOR = 7;
    public static final int FRONT_RIGHT_ROTATION_SENSOR = 11;
  }

  public static final class DriverJoysticks {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int CODRIVER_CONTROLLER_PORT = 1;
  }

  /** Inner class to store IDs of the solenoids connected to PCM. */
  public static final class Pcm {}
}
