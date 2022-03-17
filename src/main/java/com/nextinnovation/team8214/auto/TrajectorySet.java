package com.nextinnovation.team8214.auto;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Pose2dWithCurvature;
import com.nextinnovation.lib.trajectory.Trajectory;
import com.nextinnovation.lib.trajectory.timing.TimedState;
import com.nextinnovation.team8214.Field;

import java.util.ArrayList;
import java.util.List;

public class TrajectorySet {
  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static TrajectorySet instance = null;

  public static synchronized TrajectorySet getInstance() {
    if (instance == null) {
      instance = new TrajectorySet();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  public final Trajectory<TimedState<Pose2dWithCurvature>> topStartToTopBall;

  public final Trajectory<TimedState<Pose2dWithCurvature>> bottomStartToBottomBall;
  public final Trajectory<TimedState<Pose2dWithCurvature>> bottomBallToMidBall;
  public final Trajectory<TimedState<Pose2dWithCurvature>> midBallToHumanStationBall;
  public final Trajectory<TimedState<Pose2dWithCurvature>> humanStationBallToEndRobotShootPoint;

  private TrajectorySet() {
    // Top
    topStartToTopBall = getTopStartToTopBall();

    // Bottom
    bottomStartToBottomBall = getBottomStartToBottomBall();
    bottomBallToMidBall = getBottomBallToMidBall();
    midBallToHumanStationBall = getMidBallToHumanStationBall();
    humanStationBallToEndRobotShootPoint = getHumanStationBallToEndRobotShootPoint();
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  private Trajectory<TimedState<Pose2dWithCurvature>> getTopStartToTopBall() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.CriticalWaypoints.TOP_START_ROBOT_POSE);
    waypoints.add(Field.CriticalWaypoints.TOP_BALL_COLLECT_POSE);

    // ! Since trajectory generator can't set swerve kinematics constraint, so the max translation
    // ! velocity should be adjusted manually.
    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 50.0);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getBottomStartToBottomBall() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.CriticalWaypoints.BOTTOM_START_ROBOT_POSE);
    waypoints.add(Field.CriticalWaypoints.BOTTOM_BALL_COLLECT_POSE);

    // ! Since trajectory generator can't set swerve kinematics constraint, so the max translation
    // ! velocity should be adjusted manually.
    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 110.0);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getBottomBallToMidBall() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.CriticalWaypoints.BOTTOM_BALL_COLLECT_FLIPPED_POSE);
    waypoints.add(Field.CriticalWaypoints.MID_BALL_COLLECT_POSE);

    // ! Since trajectory generator can't set swerve kinematics constraint, so the max translation
    // ! velocity should be adjusted manually.
    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 90.0);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getMidBallToHumanStationBall() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.CriticalWaypoints.MID_BALL_COLLECT_POSE);
    waypoints.add(Field.CriticalWaypoints.HUMAN_STATION_BALL_COLLECT_POSE);

    // ! Since trajectory generator can't set swerve kinematics constraint, so the max translation
    // ! velocity should be adjusted manually.
    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 110.0);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getHumanStationBallToEndRobotShootPoint() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.CriticalWaypoints.HUMAN_STATION_BALL_COLLECT_FLIPPED_POSE);
    waypoints.add(Field.CriticalWaypoints.BOTTOM_END_ROBOT_SHOOT_FLIPPED_POSE);

    // ! Since trajectory generator can't set swerve kinematics constraint, so the max translation
    // ! velocity should be adjusted manually.
    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 150.0);
  }
}
