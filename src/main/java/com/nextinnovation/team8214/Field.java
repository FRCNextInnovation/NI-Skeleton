package com.nextinnovation.team8214;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;

public final class Field {
  // Field Edge
  public static final double Y_MAX = 8.235994;
  public static final double Y_MIN = 0.0;
  public static final double X_MAX = 16.4592;
  public static final double X_MIN = 0.0;

  public static final double X_MID = X_MIN + (X_MAX - X_MIN) / 2.0;
  public static final double Y_MID = Y_MIN + (Y_MAX - Y_MIN) / 2.0;

  // Physical Dimensions of Field Objects
  public static final Rotation2d FLIP = new Rotation2d(-1.0, 0.0, false);
  public static final double VISUAL_TARGET_VISUAL_CENTER_HEIGHT = 3.0;

  public static final class OriginalWaypoints {
    public static final Pose2d TOP_START_ROBOT_POSITION =
        new Pose2d(new Translation2d(4.0, 5.0), new Rotation2d(90.0));
    public static final Pose2d TOP_BALL_POSITION =
        new Pose2d(new Translation2d(4.0, 6.0), new Rotation2d(180.0));
    public static final Pose2d TOP_END_BALL_POSITION =
        new Pose2d(new Translation2d(4.0, 7.0), new Rotation2d(90.0));
  }

  // CRITICAL POSES
  // Origin is the on the right of the field center.
  // +y is towards the center of the field.
  // +x is to the right.
  public static final class CriticalWaypoints {
    public static final Pose2d TOP_START_ROBOT_POSE = OriginalWaypoints.TOP_START_ROBOT_POSITION;
    public static final Pose2d TOP_BALL_COLLECT_POSE = OriginalWaypoints.TOP_BALL_POSITION;
    public static final Pose2d TOP_BALL_COLLECT_FLIPPED_POSE =
        TOP_BALL_COLLECT_POSE.rotationByOtherWithOwnCenter(new Rotation2d(-180.0));
    public static final Pose2d TOP_END_BALL_REJECT_POSITION =
        OriginalWaypoints.TOP_END_BALL_POSITION;
  }
}
