package com.nextinnovation.team8214.auto;

import com.nextinnovation.lib.auto.modes.BaseAutoMode;
import com.nextinnovation.team8214.auto.modes.Bottom5Balls;
import com.nextinnovation.team8214.auto.modes.Silence;
import com.nextinnovation.team8214.auto.modes.Top2Balls;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeChooser {
  private static AutoModeChooser instance = null;

  public static AutoModeChooser getInstance() {
    if (instance == null) {
      instance = new AutoModeChooser();
    }
    return instance;
  }

  public enum Alliance {
    RED,
    BLUE
  }

  public enum AutoOption {
    SILENCE("Silence"),
    BOTTOM_5_BALLS("Bottom5Balls"),
    TOP_2_BALLS("Top2Balls");

    public final String name;

    AutoOption(String name) {
      this.name = name;
    }
  }

  private static final AutoOption DEFAULT_MODE = AutoOption.BOTTOM_5_BALLS;
  private final SendableChooser<AutoOption> modeChooser;
  private final SendableChooser<Alliance> allianceChooser;

  private AutoModeChooser() {
    // Mode Chooser
    modeChooser = new SendableChooser<>();
    modeChooser.setDefaultOption(AutoOption.BOTTOM_5_BALLS.name, AutoOption.BOTTOM_5_BALLS);
    modeChooser.addOption(AutoOption.SILENCE.name, AutoOption.SILENCE);
    modeChooser.addOption(AutoOption.TOP_2_BALLS.name, AutoOption.TOP_2_BALLS);

    // Alliance Chooser
    allianceChooser = new SendableChooser<>();
    allianceChooser.setDefaultOption("Blue", Alliance.BLUE);
    allianceChooser.addOption("Red", Alliance.RED);

    SmartDashboard.putData("Mode Chooser", modeChooser);
    SmartDashboard.putData("Alliance Chooser", allianceChooser);
    SmartDashboard.putString("Selected Auto Mode", DEFAULT_MODE.name);
  }

  public boolean isStandardCarpetSide() {
    return allianceChooser.getSelected() == Alliance.BLUE;
  }

  public BaseAutoMode getSelectedAutoMode() {
    AutoOption selectedOption = modeChooser.getSelected();

    return createAutoMode(selectedOption);
  }

  private BaseAutoMode createAutoMode(AutoOption option) {
    switch (option) {
      case SILENCE:
        return new Silence();
      case BOTTOM_5_BALLS:
        return new Bottom5Balls();
      case TOP_2_BALLS:
        return new Top2Balls();
      default:
        System.out.println("ERROR: unexpected auto mode: " + option);
        return null;
    }
  }

  public void logToSmartDashboard() {
    SmartDashboard.putString("Selected Alliance", allianceChooser.getSelected().name());
    SmartDashboard.putString("Selected Auto Mode", modeChooser.getSelected().name);
  }
}
