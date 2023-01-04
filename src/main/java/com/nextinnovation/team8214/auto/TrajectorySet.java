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

  public final Trajectory<TimedState<Pose2dWithCurvature>> topBallToTopReject;

  private TrajectorySet() {
    // Top
    topStartToTopBall = getTopStartToTopBall();
    topBallToTopReject = getTopBallToTopReject();
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
    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 1.27);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getTopBallToTopReject() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.CriticalWaypoints.TOP_BALL_COLLECT_FLIPPED_POSE);
    waypoints.add(Field.CriticalWaypoints.TOP_END_BALL_REJECT_POSITION);

    // ! Since trajectory generator can't set swerve kinematics constraint, so the max translation
    // ! velocity should be adjusted manually.
    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 1.27);
  }
}
