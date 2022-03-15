package com.nextinnovation.team8214.auto.modes;

import com.nextinnovation.lib.auto.AutoModeEndedException;
import com.nextinnovation.lib.auto.modes.BaseAutoMode;
import com.nextinnovation.team8214.Field;
import com.nextinnovation.team8214.auto.TrajectorySet;
import com.nextinnovation.team8214.auto.actions.ResetPoseAction;
import com.nextinnovation.team8214.auto.actions.SetStandardCarpetSideAction;
import com.nextinnovation.team8214.auto.actions.SetTrajectoryAction;
import com.nextinnovation.team8214.auto.actions.WaitAction;

public class Top2Balls extends BaseAutoMode {
  private final TrajectorySet trajectorySet = TrajectorySet.getInstance();

  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new ResetPoseAction(Field.CriticalWaypoints.TOP_START_ROBOT_POSE, true));
    runAction(new SetStandardCarpetSideAction());
    runAction(new WaitAction(0.2));
    runAction(new SetTrajectoryAction(trajectorySet.topStartToTopBall, -90.0));
  }
}
