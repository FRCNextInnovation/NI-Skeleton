package com.nextinnovation.team8214.auto;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Pose2dWithCurvature;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.trajectory.DistanceView;
import com.nextinnovation.lib.trajectory.Trajectory;
import com.nextinnovation.lib.trajectory.TrajectoryUtil;
import com.nextinnovation.lib.trajectory.timing.ITimingConstraint;
import com.nextinnovation.lib.trajectory.timing.TimedState;
import com.nextinnovation.lib.trajectory.timing.TimingUtil;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryGenerator {
  public static final class TrajectoryGeneratorConfig {
    public static final double MAX_DELTA_X = 2.0;
    public static final double MAX_DELTA_Y = 0.25;
    public static final double MAX_DELTA_THETA = Math.toRadians(5.0);

    public static final double MAX_ABS_ACCEL = 120.0;
  }

  public enum TrajectorySplineType {
    QUINTIC_HERMITE_SPLINE
  }

  public static Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
      TrajectorySplineType spline_type,
      boolean need_reversed,
      final List<Pose2d> waypoints,
      final List<ITimingConstraint<Pose2dWithCurvature>> constraints,
      double start_vel,
      double end_vel,
      double max_vel, // inches/s
      double max_accel, // inches/s^2
      double default_vel) {
    List<Pose2d> waypointsMaybeFlipped = waypoints;
    final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
    // TODO re-architect the spline generator to support reverse.
    if (need_reversed) {
      waypointsMaybeFlipped = new ArrayList<>(waypoints.size());
      for (Pose2d waypoint : waypoints) {
        waypointsMaybeFlipped.add(waypoint.transformBy(flip));
      }
    }

    // Create a trajectory from splines.
    Trajectory<Pose2dWithCurvature> trajectory;
    switch (spline_type) {
      case QUINTIC_HERMITE_SPLINE:
      default:
        trajectory =
            TrajectoryUtil.trajectoryFromSplineWaypoints(
                waypointsMaybeFlipped,
                TrajectoryGeneratorConfig.MAX_DELTA_X,
                TrajectoryGeneratorConfig.MAX_DELTA_Y,
                TrajectoryGeneratorConfig.MAX_DELTA_THETA);
        break;
    }

    if (need_reversed) {
      List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
      for (int i = 0; i < trajectory.length(); ++i) {
        flipped.add(
            new Pose2dWithCurvature(
                trajectory.getState(i).getPose().transformBy(flip),
                -trajectory.getState(i).getCurvature(),
                trajectory.getState(i).getDCurvatureDs()));
      }
      trajectory = new Trajectory<>(flipped);
    }

    List<ITimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
    if (constraints != null) {
      all_constraints.addAll(constraints);
    }
    // Generate the timed trajectory.
    Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory =
        TimingUtil.timeParameterizeTrajectory(
            need_reversed,
            new DistanceView<>(trajectory),
            TrajectoryGeneratorConfig.MAX_DELTA_X,
            all_constraints,
            start_vel,
            end_vel,
            max_vel,
            max_accel);
    timed_trajectory.setDefaultVelocity(default_vel);
    return timed_trajectory;
  }

  public static Trajectory<TimedState<Pose2dWithCurvature>> generateSwerveTrajectory(
      TrajectorySplineType spline_type,
      boolean need_reversed,
      final List<Pose2d> waypoints,
      double max_translation_vel,
      double default_vel) {
    return generateTrajectory(
        spline_type,
        need_reversed,
        waypoints,
        List.of(),
        0.0,
        0.0,
        max_translation_vel,
        TrajectoryGeneratorConfig.MAX_ABS_ACCEL,
        default_vel);
  }
}
