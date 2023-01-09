package com.nextinnovation.team8214.auto;

import com.nextinnovation.lib.auto.modes.BaseAutoMode;
import com.nextinnovation.team8214.auto.modes.Silence;
import com.nextinnovation.team8214.auto.modes.Top2Balls;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeChooser {
  private static AutoModeChooser instance = null;

  public static synchronized AutoModeChooser getInstance() {
    if (instance == null) {
      instance = new AutoModeChooser();
    }
    return instance;
  }

  public enum AutoOption {
    SILENCE("Silence"),
    TOP_2_BALLS("Top2Balls");

    public final String name;

    AutoOption(String name) {
      this.name = name;
    }
  }

  private static final AutoOption DEFAULT_MODE = AutoOption.TOP_2_BALLS;
  private final SendableChooser<AutoOption> modeChooser;
  private AutoOption selectedOption;
  private AutoOption lastSelectedOption;

  private BaseAutoMode autoMode;

  private AutoModeChooser() {
    // Mode Chooser
    modeChooser = new SendableChooser<>();
    modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
    modeChooser.addOption(AutoOption.SILENCE.name, AutoOption.SILENCE);

    selectedOption = DEFAULT_MODE;
    lastSelectedOption = null;
    autoMode = null;
  }

  public synchronized void updateSelectedAutoMode() {
    selectedOption = modeChooser.getSelected();

    if (selectedOption == null) {
      selectedOption = AutoOption.SILENCE;
    }

    if (lastSelectedOption != selectedOption) {
      System.out.println("Auto option selected -> " + selectedOption.name);
      autoMode = createAutoMode(selectedOption);
    }

    lastSelectedOption = selectedOption;
  }

  public synchronized BaseAutoMode getSelectedAutoMode() {
    return autoMode;
  }

  private BaseAutoMode createAutoMode(AutoOption option) {
    switch (option) {
      case SILENCE:
        return new Silence();
      case TOP_2_BALLS:
        return new Top2Balls();
      default:
        System.out.println("ERROR: unexpected auto mode: " + option);
        return null;
    }
  }

  public void logToSmartDashboard() {
    SmartDashboard.putData("Mode Chooser", modeChooser);
    SmartDashboard.putString("Selected Auto Mode", selectedOption.name);
  }
}
