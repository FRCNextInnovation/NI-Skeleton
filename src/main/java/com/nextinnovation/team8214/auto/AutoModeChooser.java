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

  public enum Alliance {
    RED("Red"),
    BLUE("Blue");

    public final String name;

    Alliance(String name) {
      this.name = name;
    }
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
  private static final Alliance DEFAULT_ALLIANCE = Alliance.RED;
  private final SendableChooser<AutoOption> modeChooser;
  private final SendableChooser<Alliance> allianceChooser;
  private AutoOption selectedOption;
  private Alliance selectedAlliance;

  private AutoModeChooser() {
    // Mode Chooser
    modeChooser = new SendableChooser<>();
    modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
    modeChooser.addOption(AutoOption.SILENCE.name, AutoOption.SILENCE);

    // Alliance Chooser
    allianceChooser = new SendableChooser<>();
    allianceChooser.setDefaultOption(DEFAULT_ALLIANCE.name, DEFAULT_ALLIANCE);
    allianceChooser.addOption(Alliance.BLUE.name, Alliance.BLUE);

    selectedOption = DEFAULT_MODE;
    selectedAlliance = DEFAULT_ALLIANCE;
  }

  public synchronized void updateSelectedAutoMode() {
    selectedOption = modeChooser.getSelected();
    selectedAlliance = allianceChooser.getSelected();
  }

  public BaseAutoMode getSelectedAutoMode() {
    return createAutoMode(selectedOption);
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

  public boolean isAllianceRed() {
    return selectedAlliance == Alliance.RED;
  }

  public void logToSmartDashboard() {
    SmartDashboard.putData("Mode Chooser", modeChooser);
    SmartDashboard.putData("Alliance Chooser", allianceChooser);
    SmartDashboard.putString("Selected Alliance", selectedAlliance.name);
    SmartDashboard.putString("Selected Auto Mode", selectedOption.name);
  }
}
