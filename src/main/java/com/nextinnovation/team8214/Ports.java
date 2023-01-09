package com.nextinnovation.team8214;

import com.nextinnovation.lib.drivers.CanId;

/** Final class to store all ID of CAN devices or the solenoids connected to PCM. */
public final class Ports {
  /** Inner class to store IDs of CAN devices. */
  public static final class Can {
    public static final String RIO_BUS = "rio";
    public static final String CANIVORE_BUS = "canivore";

    public static final CanId PIGEON_CHASSIS = new CanId(0, CANIVORE_BUS);

    // Swerve Drive
    public static final CanId FRONT_LEFT_DRIVE_MOTOR = new CanId(0, CANIVORE_BUS);
    public static final CanId FRONT_LEFT_ROTATION_MOTOR = new CanId(1, CANIVORE_BUS);
    public static final CanId FRONT_LEFT_ROTATION_SENSOR = new CanId(8, CANIVORE_BUS);

    public static final CanId REAR_LEFT_DRIVE_MOTOR = new CanId(2, CANIVORE_BUS);
    public static final CanId REAR_LEFT_ROTATION_MOTOR = new CanId(3, CANIVORE_BUS);
    public static final CanId REAR_LEFT_ROTATION_SENSOR = new CanId(9, CANIVORE_BUS);

    public static final CanId REAR_RIGHT_DRIVE_MOTOR = new CanId(4, CANIVORE_BUS);
    public static final CanId REAR_RIGHT_ROTATION_MOTOR = new CanId(5, CANIVORE_BUS);
    public static final CanId REAR_RIGHT_ROTATION_SENSOR = new CanId(10, CANIVORE_BUS);

    public static final CanId FRONT_RIGHT_DRIVE_MOTOR = new CanId(6, CANIVORE_BUS);
    public static final CanId FRONT_RIGHT_ROTATION_MOTOR = new CanId(7, CANIVORE_BUS);
    public static final CanId FRONT_RIGHT_ROTATION_SENSOR = new CanId(11, CANIVORE_BUS);
  }

  public static final class DriverJoysticks {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int CODRIVER_CONTROLLER_PORT = 1;
  }

  /** Inner class to store IDs of the solenoids connected to PCM. */
  public static final class Pcm {}
}
