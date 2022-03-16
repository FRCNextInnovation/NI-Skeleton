package com.nextinnovation.team8214.subsystems.swerve;

import com.nextinnovation.lib.controllers.HeadingController;
import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Pose2dWithCurvature;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.kinematics.SwerveInverseKinematics;
import com.nextinnovation.lib.kinematics.SwerveKinematics;
import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.lib.planners.DriveMotionPlanner;
import com.nextinnovation.lib.trajectory.TimedView;
import com.nextinnovation.lib.trajectory.Trajectory;
import com.nextinnovation.lib.trajectory.TrajectoryIterator;
import com.nextinnovation.lib.trajectory.timing.TimedState;
import com.nextinnovation.lib.utils.Util;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Ports;
import com.nextinnovation.team8214.devices.ahrs.AhrsNavX;
import com.nextinnovation.team8214.devices.ahrs.BaseAhrs;
import com.nextinnovation.team8214.managers.ControlSignalManager;
import com.nextinnovation.team8214.managers.OdometryFusingManager;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Swerve extends BaseSubsystem {
  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            setState(SwerveState.MANUAL);
          }

          @Override
          public void onLoop(double timestamp) {
            synchronized (stateLock) {
              updateOdometer(timestamp);

              switch (swerveState) {
                case MANUAL:
                  Translation2d translationalInput =
                      ControlSignalManager.getInstance().getSwerveManualTranslation();
                  double rotationalInput =
                      ControlSignalManager.getInstance().getSwerveManualRotationMagnitude();
                  if (Util.epsilonEquals(rotationalInput, 0.0)) {
                    if (!isHeadingControllerEnabled()
                        && Math.abs(getAngularVelocity().getUnboundedDegrees()) <= 5.625) {
                      setTargetHeadingToCurrentHeading();
                      enableHeadingController();
                    }
                  } else {
                    disableHeadingController();
                  }
                  updateNormalizedVectorialVelocityControl(
                      translationalInput, rotationalInput, false, timestamp);
                  break;

                case TRAJECTORY:
                  if (!driveMotionPlanner.isDone()) {
                    Translation2d driveVector = driveMotionPlanner.update(getPose());
                    double maxRotationMagnitude =
                        driveMotionPlanner.getNormalizedMaxRotationSpeed();

                    updateNormalizedTranslationVelocityControl(
                        driveVector, maxRotationMagnitude, true, timestamp);
                  } else {
                    return;
                  }
                  break;

                case DISABLE:
                  disableModules();
                  break;
              }
            }
          }

          @Override
          public void onStop(double timestamp) {
            disable();
          }
        };
    enabledLooper.register(loop);
  }

  /***********************************************************************************************
   * Periodic IO *
   ***********************************************************************************************/
  private static class PeriodicInput {
    public Rotation2d ahrsHeading = new Rotation2d();
    public Rotation2d ahrsAngularVelocity = new Rotation2d();
  }

  @Override
  public synchronized void readPeriodicInputs() {
    periodicInput.ahrsHeading = ahrs.getRobotHeading();
    periodicInput.ahrsAngularVelocity = ahrs.getRobotAngularVelocity();
    modules.forEach(SwerveDriveModule::readPeriodicInputs);
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    modules.forEach(SwerveDriveModule::writePeriodicOutputs);
  }

  /***********************************************************************************************
   * Subsystem States *
   ***********************************************************************************************/
  private SwerveState swerveState;

  private final Object stateLock = new Object();

  public synchronized void setState(SwerveState new_state) {
    synchronized (stateLock) {
      swerveState = new_state;

      switch (swerveState) {
        case MANUAL:
          configHeadingController(
              SwerveConfig.HeadingController.Manual.KP,
              SwerveConfig.HeadingController.Manual.KI,
              SwerveConfig.HeadingController.Manual.KD,
              SwerveConfig.HeadingController.Manual.ERROR_TOLERANCE);
          break;
        case DISABLE:
          disableHeadingController();
          disableModules();
          break;
        case TRAJECTORY:
          configHeadingController(
              SwerveConfig.HeadingController.Auto.KP,
              SwerveConfig.HeadingController.Auto.KI,
              SwerveConfig.HeadingController.Auto.KD,
              SwerveConfig.HeadingController.Auto.ERROR_TOLERANCE);
          enableHeadingController();
          break;
      }
    }
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static Swerve instance = null;

  public static synchronized Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private Pose2d startingPose = new Pose2d();

  private final PeriodicInput periodicInput = new PeriodicInput();
  private final SwerveInverseKinematics inverseKinematics =
      new SwerveInverseKinematics(
          SwerveConfig.MODULE_COUNT, SwerveConfig.POSITIONS_RELATIVE_TO_DRIVE_CENTER);
  private final SwerveKinematics kinematics =
      new SwerveKinematics(SwerveConfig.WHEEL_ODOMETER_MAX_DELTA_INCH);
  private final HeadingController headingController = new HeadingController();
  private final SwerveDriveModule frontRightModule;
  private final SwerveDriveModule frontLeftModule;
  private final SwerveDriveModule rearLeftModule;
  private final SwerveDriveModule rearRightModule;
  private final List<SwerveDriveModule> modules;
  private final List<SwerveDriveModule> odometerModules;
  private final OdometryFusingManager odometryFusingManager = OdometryFusingManager.getInstance();
  private final DriveMotionPlanner driveMotionPlanner =
      new DriveMotionPlanner(
          0.25,
          6.0,
          SwerveConfig.MAX_SPEED_INCHES_PER_SECOND,
          0.3,
          DriveMotionPlanner.FollowerType.ADAPTIVE_PURE_PURSUIT);
  //  private final BaseAhrs ahrs = AhrsPigeon.getInstance();
  private final BaseAhrs ahrs = AhrsNavX.getInstance();

  private Swerve() {
    frontLeftModule =
        new SwerveDriveModule(
            0,
            Ports.Can.FRONT_LEFT_DRIVE_MOTOR,
            Ports.Can.FRONT_LEFT_ROTATION_MOTOR,
            Ports.Can.FRONT_LEFT_ROTATION_SENSOR,
            SwerveConfig.FRONT_LEFT_CALIBRATION_OFFSET,
            SwerveConfig.FRONT_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER);
    rearLeftModule =
        new SwerveDriveModule(
            1,
            Ports.Can.REAR_LEFT_DRIVE_MOTOR,
            Ports.Can.REAR_LEFT_ROTATION_MOTOR,
            Ports.Can.REAR_LEFT_ROTATION_SENSOR,
            SwerveConfig.REAR_LEFT_CALIBRATION_OFFSET,
            SwerveConfig.REAR_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER);
    rearRightModule =
        new SwerveDriveModule(
            2,
            Ports.Can.REAR_RIGHT_DRIVE_MOTOR,
            Ports.Can.REAR_RIGHT_ROTATION_MOTOR,
            Ports.Can.REAR_RIGHT_ROTATION_SENSOR,
            SwerveConfig.REAR_RIGHT_CALIBRATION_OFFSET,
            SwerveConfig.REAR_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER);
    frontRightModule =
        new SwerveDriveModule(
            3,
            Ports.Can.FRONT_RIGHT_DRIVE_MOTOR,
            Ports.Can.FRONT_RIGHT_ROTATION_MOTOR,
            Ports.Can.FRONT_RIGHT_ROTATION_SENSOR,
            SwerveConfig.FRONT_RIGHT_CALIBRATION_OFFSET,
            SwerveConfig.FRONT_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER);

    modules = Arrays.asList(frontLeftModule, rearLeftModule, rearRightModule, frontRightModule);
    odometerModules = modules;

    configModules();
    resetSensors();
    setState(SwerveState.DISABLE);
  }

  private synchronized void configModules() {
    frontRightModule.enableTranslationInverted(true);
    rearRightModule.enableTranslationInverted(true);
  }

  public synchronized void configHeadingController(
      double kp, double ki, double kd, double errorTolerance) {
    headingController.configParmas(kp, ki, kd, errorTolerance);
  }

  /************************************************************************************************
   * Reset *
   ************************************************************************************************/
  public synchronized void resetPose() {
    setPose(startingPose);
    odometryFusingManager.reset(Timer.getFPGATimestamp(), startingPose);
    kinematics.resetTotalDistance();
  }

  @Override
  public synchronized void resetSensors() {
    resetPose();
  }

  /************************************************************************************************
   * Function Enabler *
   ************************************************************************************************/
  public void enableHeadingController() {
    headingController.enable();
  }

  public void disableHeadingController() {
    headingController.disable();
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public synchronized void setIsStandardCarpetSide(boolean is_standard_carpet_side) {
    modules.forEach(s -> s.setIsStandardCarpetSide(is_standard_carpet_side));
  }

  public Rotation2d getTargetHeading() {
    return headingController.getTargetHeading();
  }

  public Rotation2d getFieldCentricHeading() {
    return periodicInput.ahrsHeading;
  }

  public Pose2d getPose() {
    return odometryFusingManager.getLatestFieldCentricRobotPose();
  }

  public synchronized void setPose(Pose2d pose) {
    kinematics.setPose(pose);
    modules.forEach((module) -> module.setModulePositionFromRobotPose(pose));
  }

  public void setStartingPose(Pose2d pose) {
    startingPose = pose;
  }

  public double getTotalDistance() {
    return kinematics.getTotalDistance();
  }

  public Translation2d getVelocity() {
    return kinematics.getVelocity();
  }

  public Rotation2d getAngularVelocity() {
    return periodicInput.ahrsAngularVelocity;
  }

  public synchronized void setTargetHeading(Rotation2d targetAbsoluteHeadingDegrees) {
    headingController.setTargetHeading(targetAbsoluteHeadingDegrees);
  }

  public synchronized void setTargetHeading(double targetAbsoluteHeadingDegrees) {
    headingController.setTargetHeading(
        new Rotation2d(Util.boundAngleTo0To360Degrees(targetAbsoluteHeadingDegrees)));
  }

  public synchronized void setTargetHeadingToCurrentHeading() {
    setTargetHeading(periodicInput.ahrsHeading);
  }

  public synchronized void setModulesAlign(Rotation2d heading, boolean isFieldCentric) {
    List<Rotation2d> headings = new ArrayList<>(modules.size());
    for (int i = 0; i < modules.size(); i++) {
      headings.add(
          isFieldCentric ? heading.rotateBy(periodicInput.ahrsHeading.inverse()) : heading);
    }
    setModuleHeadingTargets(headings);
  }

  private synchronized void setNormalizedModuleVelocityTargets(
      List<Translation2d> moduleVelocities, boolean enableClosedLoopControl) {
    for (int i = 0; i < modules.size(); i++) {
      SwerveDriveModule module = modules.get(i);
      Translation2d moduleVelocity = moduleVelocities.get(i);
      if (Util.shouldReverseRotation(
          moduleVelocities.get(i).direction().getDegrees(),
          modules.get(i).getRobotCentricRotationHeading().getDegrees())) {
        if (!Util.epsilonEquals(moduleVelocity.norm(), 0.0)) {
          module.setRotationHeadingTarget(
              moduleVelocity.direction().rotateBy(Rotation2d.fromDegrees(180.0)));
        }
        if (enableClosedLoopControl) {
          module.setNormalizedTranslationVelocityTarget(-moduleVelocity.norm());
        } else {
          module.setTranslationOpenLoop(-moduleVelocity.norm());
        }
      } else {
        if (!Util.epsilonEquals(moduleVelocity.norm(), 0.0)) {
          module.setRotationHeadingTarget(moduleVelocity.direction());
        }
        if (enableClosedLoopControl) {
          module.setNormalizedTranslationVelocityTarget(moduleVelocity.norm());
        } else {
          module.setTranslationOpenLoop(moduleVelocity.norm());
        }
      }
    }
  }

  public synchronized void setTrajectory(
      Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
      Translation2d followingCenter,
      double targetHeading) {
    setState(SwerveState.TRAJECTORY);
    setTargetHeading(targetHeading);
    driveMotionPlanner.reset();
    driveMotionPlanner.setFollowingCenter(followingCenter);
    driveMotionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
  }

  public synchronized void setTrajectory(
      Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double goalHeading) {
    System.out.println("setTrajectory enter swerve class");
    setTrajectory(trajectory, Translation2d.identity(), goalHeading);
  }

  public boolean isDoneWithTrajectory() {
    if (swerveState != SwerveState.TRAJECTORY) {
      return false;
    } else {
      return driveMotionPlanner.isDone();
    }
  }

  public boolean isHeadingOnTarget() {
    return headingController.onTarget();
  }

  public boolean isHeadingControllerEnabled() {
    return headingController.isEnabled();
  }

  public boolean isModuleVelocitiesOnTarget() {
    for (SwerveDriveModule module : modules) {
      if (!module.isTranslationVelocityOnTarget()) {
        return false;
      }
    }
    return true;
  }

  private synchronized void setModuleHeadingTargets(List<Rotation2d> moduleRotationHeadings) {
    for (int i = 0; i < modules.size(); i++) {
      if (Util.shouldReverseRotation(
          moduleRotationHeadings.get(i).getDegrees(),
          modules.get(i).getRobotCentricRotationHeading().getDegrees())) {
        modules
            .get(i)
            .setRotationHeadingTarget(
                moduleRotationHeadings.get(i).rotateBy(Rotation2d.fromDegrees(180.0)));
      } else {
        modules.get(i).setRotationHeadingTarget(moduleRotationHeadings.get(i));
      }
    }
  }

  public boolean isModuleHeadingsOnTarget() {
    for (SwerveDriveModule module : modules) {
      if (!module.isRotationHeadingOnTarget()) {
        return false;
      }
    }
    return true;
  }

  /************************************************************************************************
   * Update *
   ************************************************************************************************/
  public synchronized void updateOdometer(double timestamp) {
    List<Pose2d> moduleEstimatedRobotPoses = new ArrayList<>();
    for (SwerveDriveModule module : odometerModules) {
      module.updateOdometer(periodicInput.ahrsHeading);
      moduleEstimatedRobotPoses.add(module.getEstimatedRobotPose());
    }
    kinematics.update(moduleEstimatedRobotPoses, periodicInput.ahrsHeading, timestamp);
    odometryFusingManager.addChassisWODeltaTranslationMap(
        timestamp, kinematics.getDeltaPosition(), periodicInput.ahrsHeading);
    odometerModules.forEach(
        (module) -> module.setModulePositionFromRobotPose(kinematics.getPose()));
  }

  public synchronized void updateNormalizedVectorialVelocityControl(
      Translation2d translationVector,
      double rotationMagnitude,
      boolean enableClosedLoopControl,
      double timestamp) {
    setNormalizedModuleVelocityTargets(
        inverseKinematics.calculateNormalizedModuleVelocities(
            translationVector,
            rotationMagnitude + headingController.calculate(getFieldCentricHeading(), timestamp),
            getFieldCentricHeading()),
        enableClosedLoopControl);
  }

  public synchronized void updateNormalizedTranslationVelocityControl(
      Translation2d translationVector,
      double maxRotationMagnitude,
      boolean enableClosedLoopControl,
      double timestamp) {
    setNormalizedModuleVelocityTargets(
        inverseKinematics.calculateNormalizedModuleVelocities(
            translationVector,
            Util.limit(
                headingController.calculate(getFieldCentricHeading(), timestamp),
                maxRotationMagnitude),
            getFieldCentricHeading()),
        enableClosedLoopControl);
  }

  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  public void disableModules() {
    modules.forEach(SwerveDriveModule::disable);
  }

  @Override
  public void disable() {
    setState(SwerveState.DISABLE);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  @Override
  public void logToSmartDashboard() {
    SmartDashboard.putString("Swerve State", swerveState.value);
    SmartDashboard.putString("Robot Pose", getPose().toString());
    if (Config.ENABLE_DEBUG_OUTPUT) {
      SmartDashboard.putBoolean("Is HeadingController Enabled", isHeadingControllerEnabled());
      SmartDashboard.putNumber("Total Distance Travelled", getTotalDistance());
      SmartDashboard.putNumber("Target Heading", getTargetHeading().getDegrees());
      SmartDashboard.putBoolean("Is Heading on Target", isHeadingOnTarget());
      SmartDashboard.putNumber("IMU Fused Heading", getFieldCentricHeading().getDegrees());
      SmartDashboard.putBoolean("Is Module Velocities On Target", isModuleVelocitiesOnTarget());
      SmartDashboard.putBoolean("Is Module Headings On Target", isModuleHeadingsOnTarget());
      SmartDashboard.putBoolean("Is Trajectory Finished", isDoneWithTrajectory());
    }
    modules.forEach(SwerveDriveModule::logToSmartDashboard);
  }

  @Override
  public boolean selfTest() {
    boolean passesTest = true;
    for (SwerveDriveModule module : modules) {
      passesTest &= module.selfTest();
    }
    return passesTest;
  }
}
