package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseAction;
import com.nextinnovation.lib.geometry.Pose2dWithCurvature;
import com.nextinnovation.lib.trajectory.Trajectory;
import com.nextinnovation.lib.trajectory.timing.TimedState;
import com.nextinnovation.team8214.subsystems.swerve.Swerve;

public class SetTrajectoryAction extends BaseAction {
  private final Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
  private final double goalHeading;
  private final Swerve swerve;

  public SetTrajectoryAction(
      Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double goalHeading) {
    this.trajectory = trajectory;
    this.goalHeading = goalHeading;
    swerve = Swerve.getInstance();
  }

  @Override
  public void start() {
    swerve.setTrajectory(trajectory, goalHeading);
    System.out.println("Trajectory start!");
  }

  @Override
  public void update() {}

  @Override
  public void done() {}

  @Override
  public boolean isFinished() {
    if (swerve.isDoneWithTrajectory()) {
      System.out.println("Trajectory finished!");
      return true;
    } else {
      return false;
    }
  }
}
