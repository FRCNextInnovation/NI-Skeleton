package com.nextinnovation.lib.auto;

import com.nextinnovation.lib.utils.BaseCrashTrackingRunnable;
import com.nextinnovation.lib.auto.modes.BaseAutoMode;

/** This class selects, runs, and stops (if necessary) a specified autonomous mode. */
public class AutoModeExecuter {
  private BaseAutoMode auto_mode;
  private Thread thread = null;

  public void setAutoMode(BaseAutoMode new_auto_mode) {
    auto_mode = new_auto_mode;
  }

  public void start() {
    if (thread == null) {
      thread =
          new Thread(
              new BaseCrashTrackingRunnable() {
                @Override
                public void runCrashTracked() {
                  if (auto_mode != null) {
                    auto_mode.run();
                  }
                }
              });

      thread.start();
    }
  }

  public void stop() {
    if (auto_mode != null) {
      auto_mode.stop();
    }

    thread = null;
  }
}
