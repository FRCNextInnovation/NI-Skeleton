package com.nextinnovation.lib.auto.actions;

import java.util.ArrayList;
import java.util.List;

public class SeriesAction extends BaseAction {
  private BaseAction curAction;
  private final ArrayList<BaseAction> remainingActions;

  public SeriesAction(List<BaseAction> actions) {
    remainingActions = new ArrayList<>(actions.size());

    // Avoid to use addAll() for safe.
    for (BaseAction action : actions) {
      remainingActions.add(action);
    }

    curAction = null;
  }

  @Override
  public boolean isFinished() {
    return remainingActions.isEmpty() && curAction == null;
  }

  @Override
  public void start() {}

  @Override
  public void update() {
    if (curAction == null) {
      if (remainingActions.isEmpty()) {
        return;
      }

      curAction = remainingActions.remove(0);
      curAction.start();
    }

    curAction.update();

    if (curAction.isFinished()) {
      curAction.done();
      curAction = null;
    }
  }

  @Override
  public void done() {}
}
