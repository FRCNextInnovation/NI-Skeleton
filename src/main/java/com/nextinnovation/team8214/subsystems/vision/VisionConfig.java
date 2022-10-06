package com.nextinnovation.team8214.subsystems.vision;

import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.utils.InterpolatingDouble;
import com.nextinnovation.lib.utils.InterpolatingTreeMap;
import com.nextinnovation.lib.utils.Units;

public class VisionConfig {
  // Shooting parameters
  public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      visionDistanceRemappedMap = new InterpolatingTreeMap<>(25);

  static {
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(1.705)),
        new InterpolatingDouble(Units.meters_to_inches(1.50)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(2.049)),
        new InterpolatingDouble(Units.meters_to_inches(1.75)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(2.359)),
        new InterpolatingDouble(Units.meters_to_inches(2.0)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(2.643)),
        new InterpolatingDouble(Units.meters_to_inches(2.25)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(3.169)),
        new InterpolatingDouble(Units.meters_to_inches(2.5)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(3.423)),
        new InterpolatingDouble(Units.meters_to_inches(2.75)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(3.759)),
        new InterpolatingDouble(Units.meters_to_inches(3.0)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(4.081)),
        new InterpolatingDouble(Units.meters_to_inches(3.25)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(4.414)),
        new InterpolatingDouble(Units.meters_to_inches(3.5)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(4.740)),
        new InterpolatingDouble(Units.meters_to_inches(3.75)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(5.095)),
        new InterpolatingDouble(Units.meters_to_inches(4.0)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(5.524)),
        new InterpolatingDouble(Units.meters_to_inches(4.25)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(5.958)),
        new InterpolatingDouble(Units.meters_to_inches(4.5)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(6.436)),
        new InterpolatingDouble(Units.meters_to_inches(4.75)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(6.870)),
        new InterpolatingDouble(Units.meters_to_inches(5.0)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(7.119)),
        new InterpolatingDouble(Units.meters_to_inches(5.25)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(7.531)),
        new InterpolatingDouble(Units.meters_to_inches(5.5)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(7.856)),
        new InterpolatingDouble(Units.meters_to_inches(5.75)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(8.264)),
        new InterpolatingDouble(Units.meters_to_inches(6.0)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(8.726)),
        new InterpolatingDouble(Units.meters_to_inches(6.25)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(9.150)),
        new InterpolatingDouble(Units.meters_to_inches(6.50)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(9.635)),
        new InterpolatingDouble(Units.meters_to_inches(6.75)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(10.187)),
        new InterpolatingDouble(Units.meters_to_inches(7.0)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(10.942)),
        new InterpolatingDouble(Units.meters_to_inches(7.25)));
    visionDistanceRemappedMap.put(
        new InterpolatingDouble(Units.meters_to_inches(11.465)),
        new InterpolatingDouble(Units.meters_to_inches(7.5)));
  }

  public static final Rotation2d CAMERA_ELEVATION = new Rotation2d(30.0);
  public static final double CAMERA_HEIGHT_INCH = 30.6321587;
}
