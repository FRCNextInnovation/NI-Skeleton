package com.nextinnovation.team8214;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;

public final class Field {
  // Physical Dimensions of Field Objects
  public static final double VISUAL_TARGET_CENTER_HEIGHT = 99.09;
  public static final double VISUAL_TARGET_VISUAL_CENTER_HEIGHT = 99.08;
  public static final double UPPER_HUB_RADIANS = 0.0;

  public static final Rotation2d FLIP = new Rotation2d(-1.0, 0.0, false);

  public static final Translation2d HUB_CENTER_POSITION = new Translation2d(-109.699, 161.746);

  public static final class OriginalWaypoints {
    public static final Pose2d TOP_START_ROBOT_POSITION =
        new Pose2d(new Translation2d(-202.238, 184.257), new Rotation2d(-90.0));
    public static final Pose2d TOP_BALL_POSITION =
        new Pose2d(new Translation2d(-241.164, 242.408), new Rotation2d(-90.0));

    public static final Pose2d BOTTOM_START_ROBOT_POSITION =
        new Pose2d(new Translation2d(-136.192, 71.876), new Rotation2d(-90.0));
    public static final Pose2d BOTTOM_BALL_POSITION =
        new Pose2d(new Translation2d(-136.192, 9.768), new Rotation2d(-90.0));
    public static final Pose2d MID_BALL_POSITION =
        new Pose2d(new Translation2d(-236.665, 72.723), new Rotation2d(165.0));
    public static final Pose2d HUMAN_STATION_BALL_POSITION =
        new Pose2d(new Translation2d(-397.623, 43.705), new Rotation2d(-135.0));
    public static final Pose2d BOTTOM_END_ROBOT_SHOOT_POSITION =
        new Pose2d(new Translation2d(-288.152, 93.872), new Rotation2d(-135.0));
  }

  // CRITICAL POSES
  // Origin is the center of the robot.
  // +x is towards the center of the field.
  // +y is to the left.
  public static final class CriticalWaypoints {
    // Top
    public static final Pose2d TOP_START_ROBOT_POSE = OriginalWaypoints.TOP_START_ROBOT_POSITION;
    public static final Pose2d TOP_BALL_COLLECT_POSE =
        OriginalWaypoints.TOP_BALL_POSITION.translateBy(
            new Translation2d(0.0, Config.INTAKE_PUSH_OUT_LENGTH + Config.ROBOT_HALF_LENGTH));

    // Bottom
    public static final Pose2d BOTTOM_START_ROBOT_POSE =
        OriginalWaypoints.BOTTOM_START_ROBOT_POSITION;

    public static final Pose2d BOTTOM_BALL_COLLECT_POSE =
        OriginalWaypoints.BOTTOM_BALL_POSITION.translateBy(
            new Translation2d(0.0, Config.ROBOT_HALF_LENGTH + Config.INTAKE_PUSH_OUT_LENGTH));

    public static final Pose2d BOTTOM_BALL_COLLECT_FLIPPED_POSE =
        BOTTOM_BALL_COLLECT_POSE.rotationByOtherWithOwnCenter(FLIP);

    public static final Pose2d MID_BALL_COLLECT_POSE =
        OriginalWaypoints.MID_BALL_POSITION.translateBy(
            Translation2d.fromPolar(
                OriginalWaypoints.MID_BALL_POSITION.getRotation().rotateBy(FLIP),
                Config.ROBOT_HALF_LENGTH + Config.INTAKE_PUSH_OUT_LENGTH));

    public static final Pose2d HUMAN_STATION_BALL_COLLECT_POSE =
        OriginalWaypoints.HUMAN_STATION_BALL_POSITION.translateBy(
            Translation2d.fromPolar(
                OriginalWaypoints.HUMAN_STATION_BALL_POSITION.getRotation().rotateBy(FLIP),
                Config.ROBOT_HALF_LENGTH + Config.INTAKE_PUSH_OUT_LENGTH));

    public static final Pose2d HUMAN_STATION_BALL_COLLECT_FLIPPED_POSE =
        HUMAN_STATION_BALL_COLLECT_POSE.rotationByOtherWithOwnCenter(FLIP);

    public static final Pose2d BOTTOM_END_ROBOT_SHOOT_POSE =
        OriginalWaypoints.BOTTOM_END_ROBOT_SHOOT_POSITION;

    public static final Pose2d BOTTOM_END_ROBOT_SHOOT_FLIPPED_POSE =
        BOTTOM_END_ROBOT_SHOOT_POSE.rotationByOtherWithOwnCenter(FLIP);
  }
}
