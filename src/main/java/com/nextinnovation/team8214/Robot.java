package com.nextinnovation.team8214;

import com.nextinnovation.lib.auto.AutoModeExecuter;
import com.nextinnovation.lib.loops.Looper;
import com.nextinnovation.lib.subsystems.SubsystemGroup;
import com.nextinnovation.lib.utils.CrashTracker;
import com.nextinnovation.lib.utils.Logger;
import com.nextinnovation.lib.wpilib.TimedRobot;
import com.nextinnovation.team8214.auto.AutoModeChooser;
import com.nextinnovation.team8214.auto.TrajectorySet;
import com.nextinnovation.team8214.devices.PneumaticCompressor;
import com.nextinnovation.team8214.devices.VideoFeeder;
import com.nextinnovation.team8214.devices.ahrs.AhrsPigeon2;
import com.nextinnovation.team8214.managers.ControlSignalManager;
import com.nextinnovation.team8214.managers.OdometerFusingManager;
import com.nextinnovation.team8214.subsystems.swerve.Swerve;
import com.nextinnovation.team8214.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import java.util.Arrays;

public class Robot extends TimedRobot {
  private final Looper controlLooper = new Looper("Control", Config.LOOPER_CONTROL_PERIOD_SEC);
  private final Looper visionLooper = new Looper("Vision", Config.LOOPER_VISION_PERIOD_SEC);

  private TrajectorySet trajectorySet;
  private AutoModeExecuter autoModeExecuter;
  private AutoModeChooser autoModeChooser;

  private ControlSignalManager controlSignalManager;
  private OdometerFusingManager odometerFusingManager;
  private PneumaticCompressor pneumaticCompressor;

  private Vision vision;
  private Swerve swerve;
  private SubsystemGroup subsystems;

  private void initManagers() {
    controlSignalManager = ControlSignalManager.getInstance();
    odometerFusingManager = OdometerFusingManager.getInstance();

    controlSignalManager.registerEnabledLoops(controlLooper);
  }

  private void initDevices() {
    AhrsPigeon2.getInstance();
    pneumaticCompressor = PneumaticCompressor.getInstance();
    pneumaticCompressor.enable();
    VideoFeeder.getInstance().enable();
  }

  private void initSubsystems() {
    vision = Vision.getInstance();

    swerve = Swerve.getInstance();

    subsystems = new SubsystemGroup(Arrays.asList(swerve));
    subsystems.registerEnabledLoops(controlLooper);

    vision.registerEnabledLoops(visionLooper);
  }

  private void initAutoTools() {
    trajectorySet = TrajectorySet.getInstance();
    autoModeChooser = AutoModeChooser.getInstance();
    autoModeExecuter = AutoModeExecuter.getInstance();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.clearLog();
    LiveWindow.disableAllTelemetry();

    try {
      initAutoTools();
      initManagers();
      initDevices();
      initSubsystems();

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
    try {
      visionLooper.start();
      controlLooper.start();

      autoModeExecuter.setAutoMode(autoModeChooser.getSelectedAutoMode());
      autoModeExecuter.start();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    try {
      if (autoModeExecuter != null) {
        autoModeExecuter.stop();
      }
      visionLooper.restart();
      controlLooper.restart();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    try {
      pneumaticCompressor.update();
      logToSmartDashboard();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    try {
      visionLooper.stop();
      controlLooper.stop();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    try {
      autoModeChooser.updateSelectedAutoMode();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    try {
      if (autoModeExecuter != null) {
        autoModeExecuter.stop();
      }
      visionLooper.restart();
      controlLooper.restart();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    teleopPeriodic();
  }

  private void logToSmartDashboard() {
    autoModeChooser.logToSmartDashboard();
    subsystems.logToSmartDashboard();
    controlLooper.logToSmartDashboard();
    controlSignalManager.logToSmartDashBoard();
    vision.logToSmartDashboard();
    odometerFusingManager.logToSmartDashBoard();
  }
}
