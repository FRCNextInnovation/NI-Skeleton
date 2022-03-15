package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseRunOnceAction;
import com.nextinnovation.team8214.auto.AutoModeChooser;
import com.nextinnovation.team8214.subsystems.swerve.Swerve;

public class SetStandardCarpetSideAction extends BaseRunOnceAction {
  private final AutoModeChooser autoModeChooser;
  private final Swerve swerve;

  public SetStandardCarpetSideAction() {
    autoModeChooser = AutoModeChooser.getInstance();
    swerve = Swerve.getInstance();
  }

  @Override
  public void runOnce() {
    swerve.setIsStandardCarpetSide(autoModeChooser.isStandardCarpetSide());
  }
}
