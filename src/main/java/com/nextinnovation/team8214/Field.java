package com.nextinnovation.team8214;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;

public final class Field {
  // Field Edge
  public static final double Y_MAX = 324.872;
  public static final double Y_MIN = 0.0;
  public static final double X_MAX = 220.214;
  public static final double X_MIN = -438.779;

  public static final double X_MID = X_MIN + (X_MAX - X_MIN) / 2.0;
  public static final double Y_MID = Y_MIN + (Y_MAX - Y_MIN) / 2.0;

  // Physical Dimensions of Field Objects
  public static final double VISUAL_TARGET_VISUAL_CENTER_HEIGHT = 99.08;
  public static final double UPPER_HUB_RADIANS = 24.0;

  public static final Rotation2d FLIP = new Rotation2d(-1.0, 0.0, false);

  public static final class OriginalWaypoints {
    public static final Translation2d HUB_CENTER_POSITION = new Translation2d(-109.699, 161.746);
    public static final Translation2d FRIEND_HANGER_POSITION = new Translation2d(-374.629, 261.343);
    public static final Translation2d ENEMY_HANGER_POSITION = new Translation2d(157.73, 54.771);

    public static final Pose2d TOP_START_ROBOT_POSITION =
        new Pose2d(new Translation2d(-202.238, 184.257), new Rotation2d(90.0));
    public static final Pose2d TOP_BALL_POSITION =
        new Pose2d(new Translation2d(-241.734, 242.899), new Rotation2d(180.0));
    public static final Pose2d TOP_END_BALL_POSITION =
        new Pose2d(new Translation2d(-199.675, 287.165), new Rotation2d(90.0));

    public static final Pose2d BOTTOM_START_ROBOT_POSITION =
        new Pose2d(new Translation2d(-138.192, 77.133), new Rotation2d(-180.0));
    public static final Pose2d BOTTOM_BALL_POSITION =
        new Pose2d(new Translation2d(-138.192, 32.636), new Rotation2d(-180.0));
    public static final Pose2d MID_BALL_POSITION =
        new Pose2d(new Translation2d(-236.782, 71.413), new Rotation2d(-180.0));
    public static final Pose2d HUMAN_STATION_BALL_POSITION =
        new Pose2d(new Translation2d(-376.197, 46.082), new Rotation2d(-135.0));
    public static final Pose2d BOTTOM_END_ROBOT_SHOOT_POSITION =
        new Pose2d(new Translation2d(-196.782, 70.133), new Rotation2d(0.0));
  }

  // CRITICAL POSES
  // Origin is the on the right of the field center.
  // +y is towards the center of the field.
  // +x is to the right.
  public static final class CriticalWaypoints {
    // Top
    public static final Pose2d TOP_START_ROBOT_POSE = OriginalWaypoints.TOP_START_ROBOT_POSITION;
    public static final Pose2d TOP_BALL_COLLECT_POSE = OriginalWaypoints.TOP_BALL_POSITION;

    public static final Pose2d TOP_BALL_COLLECT_FLIPPED_POSE =
        TOP_BALL_COLLECT_POSE.rotationByOtherWithOwnCenter(new Rotation2d(-180.0));

    public static final Pose2d TOP_END_BALL_REJECT_POSITION =
        OriginalWaypoints.TOP_END_BALL_POSITION;

    // Bottom
    public static final Pose2d BOTTOM_START_ROBOT_POSE =
        OriginalWaypoints.BOTTOM_START_ROBOT_POSITION.rotationByOtherWithOwnCenter(
            new Rotation2d(90.0));

    public static final Pose2d BOTTOM_BALL_COLLECT_POSE =
        OriginalWaypoints.BOTTOM_BALL_POSITION.rotationByOtherWithOwnCenter(new Rotation2d(90.0));

    public static final Pose2d BOTTOM_BALL_COLLECT_FLIPPED_POSE =
        OriginalWaypoints.BOTTOM_BALL_POSITION;

    public static final Pose2d MID_BALL_COLLECT_POSE = OriginalWaypoints.MID_BALL_POSITION;

    public static final Pose2d MID_BALL_COLLECT_FLIPPED_POSE = OriginalWaypoints.MID_BALL_POSITION;

    public static final Pose2d HUMAN_STATION_BALL_COLLECT_POSE =
        OriginalWaypoints.HUMAN_STATION_BALL_POSITION.translateBy(
            Translation2d.fromPolar(Rotation2d.fromDegrees(-45.0), 30.0));

    public static final Pose2d HUMAN_STATION_BALL_COLLECT_FLIPPED_POSE =
        HUMAN_STATION_BALL_COLLECT_POSE.rotationByOtherWithOwnCenter(FLIP);

    public static final Pose2d HUMAN_STATION_WAIT_FLIPPED_POSE =
        HUMAN_STATION_BALL_COLLECT_FLIPPED_POSE.translateBy(
            Translation2d.fromPolar(Rotation2d.fromDegrees(-135.0).rotateBy(FLIP), 35.0));

    public static final Pose2d BOTTOM_END_ROBOT_SHOOT_POSE =
        OriginalWaypoints.BOTTOM_END_ROBOT_SHOOT_POSITION;

    public static final Pose2d BOTTOM_END_ROBOT_SHOOT_FLIPPED_POSE =
        BOTTOM_END_ROBOT_SHOOT_POSE.rotationByOtherWithOwnCenter(FLIP);
  }
}
