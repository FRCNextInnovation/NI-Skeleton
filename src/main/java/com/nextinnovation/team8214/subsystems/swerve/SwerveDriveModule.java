package com.nextinnovation.team8214.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.nextinnovation.lib.drivers.LazyTalonFX;
import com.nextinnovation.lib.drivers.TalonUtil;
import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import com.nextinnovation.lib.utils.Util;
import com.nextinnovation.team8214.Config;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveModule extends BaseSubsystem {
  /************************************************************************************************
   * Periodic IO *
   ************************************************************************************************/
  private static class PeriodicInput {
    public int rotationMotorEncoderPosition = 0;
    public int translationMotorEncoderPosition = 0;
    public int previousTranslationMotorEncoderPosition = 0;
    public int translationMotorEncoderVelocity = 0;
  }

  private synchronized void synchronizePreviousTranslationEncoderPosition() {
    periodicInput.previousTranslationMotorEncoderPosition =
        periodicInput.translationMotorEncoderPosition;
  }

  @Override
  public synchronized void readPeriodicInputs() {
    periodicInput.rotationMotorEncoderPosition =
        Util.roundToInt(rotationMotor.getSelectedSensorPosition());
    periodicInput.translationMotorEncoderPosition =
        Util.roundToInt(translationMotor.getSelectedSensorPosition());
    periodicInput.translationMotorEncoderVelocity =
        Util.roundToInt(translationMotor.getSelectedSensorVelocity());
  }

  private static class PeriodicOutput {
    public ControlMode rotationMotorControlMode = ControlMode.PercentOutput;
    public ControlMode translationMotorControlMode = ControlMode.PercentOutput;
    public double rotationMotorSetpoint = 0.0;
    public double translationMotorSetpoint = 0.0;
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    translationMotor.set(
        periodicOutput.translationMotorControlMode, periodicOutput.translationMotorSetpoint);
    rotationMotor.set(
        periodicOutput.rotationMotorControlMode, periodicOutput.rotationMotorSetpoint);
  }

  /************************************************************************************************
   * Init & Config *
   ************************************************************************************************/
  private final LazyTalonFX translationMotor, rotationMotor;

  private final String moduleName;
  private Pose2d estimatedRobotPose = new Pose2d();
  private Translation2d modulePosition;
  private final int rotationCalibrationOffset;
  private boolean isStandardCarpetSide = false;
  private final Translation2d modulePositionToTranslationCenter;
  private final PeriodicInput periodicInput = new PeriodicInput();
  private final PeriodicOutput periodicOutput = new PeriodicOutput();
  private final CANCoder externalRotationEncoder;

  /**
   * Constructor
   *
   * @param module_id ID of module:[1, 4], usually FL -> 1, RL -> 2, RR -> 3, FR -> 4
   * @param translation_motor_id CAN ID of translation motor
   * @param rotation_motor_id CAN ID of translation motor
   * @param external_rotation_encoder_id CAN ID of CANCoder
   * @param rotation_calibration_offset Calibration offset by CANCoder
   * @param module_position_to_robot_centric Module position related to robot centric in inch
   */
  public SwerveDriveModule(
      int module_id,
      int translation_motor_id,
      int rotation_motor_id,
      int external_rotation_encoder_id,
      int rotation_calibration_offset,
      Translation2d module_position_to_robot_centric) {
    moduleName = "Swerve Module " + module_id + " ";
    translationMotor = new LazyTalonFX(translation_motor_id);
    rotationMotor = new LazyTalonFX(rotation_motor_id);
    externalRotationEncoder = new CANCoder(external_rotation_encoder_id);
    rotationCalibrationOffset = rotation_calibration_offset;
    modulePositionToTranslationCenter = module_position_to_robot_centric;
    configExternalRotationSensor();
    configTranslationMotor();
    configRotationMotor();
    calibrateRotationEncoder();
    readPeriodicInputs();
    synchronizePreviousTranslationEncoderPosition();
    resetSensors();
  }

  private synchronized void configExternalRotationSensor() {
    TalonUtil.checkError(
        externalRotationEncoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition, Config.CAN_TIMEOUT_MS),
        moduleName + " CANCoder: Can't set initialization strategy!");
    TalonUtil.checkError(
        externalRotationEncoder.configAbsoluteSensorRange(
            AbsoluteSensorRange.Unsigned_0_to_360, Config.CAN_TIMEOUT_MS),
        moduleName + " CANCoder: Can't set absolute sensor range!");
  }

  private synchronized void configTranslationMotor() {
    translationMotor.configStatusFramePeriod(
        SwerveDriveModuleConfig.Translation.STATUS_FRAME_PERIOD,
        true,
        false,
        Config.CAN_TIMEOUT_MS);

    TalonUtil.checkError(
        translationMotor.configNeutralDeadband(0.015, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set neutral deadband!");
    TalonUtil.checkError(
        translationMotor.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set feedback sensor!");
    translationMotor.setInverted(SwerveDriveModuleConfig.Translation.IS_INVERT);
    translationMotor.setNeutralMode(NeutralMode.Brake);
    TalonUtil.checkError(
        translationMotor.configVelocityMeasurementPeriod(
            SensorVelocityMeasPeriod.Period_1Ms, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set velocity measurement period!");
    TalonUtil.checkError(
        translationMotor.configVelocityMeasurementWindow(1, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set velocity measurement window!");
    TalonUtil.checkError(
        translationMotor.configOpenloopRamp(
            SwerveDriveModuleConfig.Translation.OPEN_LOOP_RAMP, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set open loop ramp!");
    TalonUtil.checkError(
        translationMotor.configVoltageCompSaturation(10.0, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set voltage comp saturation!");
    TalonUtil.checkError(
        translationMotor.configVoltageMeasurementFilter(1, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set voltage measurement filter!");
    translationMotor.enableVoltageCompensation(true);
    TalonUtil.checkError(
        translationMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 38.0, 38.0, 0.0), Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set supply current limit!");
    TalonUtil.checkError(
        translationMotor.configAllowableClosedloopError(0, 0.0, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set allowable closed loop error!");
    // PID 0 is for velocity
    TalonUtil.checkError(
        translationMotor.config_kP(
            0, SwerveDriveModuleConfig.Translation.PID0_KP, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set kp!");
    TalonUtil.checkError(
        translationMotor.config_kI(
            0, SwerveDriveModuleConfig.Translation.PID0_KI, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set ki!");
    TalonUtil.checkError(
        translationMotor.config_kD(
            0, SwerveDriveModuleConfig.Translation.PID0_KD, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set kd!");
    TalonUtil.checkError(
        translationMotor.config_kF(
            0, SwerveDriveModuleConfig.Translation.PID0_KF, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set kf!");
    TalonUtil.checkError(
        translationMotor.setSelectedSensorPosition(0, 0, Config.CAN_INSTANT_TIMEOUT_MS),
        moduleName + " Translation: Can't set selected sensor position!");

    Timer.delay(0.05);
  }

  private synchronized void configRotationMotor() {
    rotationMotor.configStatusFramePeriod(
        SwerveDriveModuleConfig.Rotation.STATUS_FRAME_PERIOD, true, false, Config.CAN_TIMEOUT_MS);
    TalonUtil.checkError(
        rotationMotor.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set feedback sensor!");
    rotationMotor.setInverted(SwerveDriveModuleConfig.Rotation.IS_INVERT);
    rotationMotor.setNeutralMode(NeutralMode.Brake);
    TalonUtil.checkError(
        rotationMotor.configVelocityMeasurementPeriod(
            SensorVelocityMeasPeriod.Period_1Ms, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set velocity measurement period!");
    TalonUtil.checkError(
        rotationMotor.configVelocityMeasurementWindow(1, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set velocity measurement window!");
    TalonUtil.checkError(
        rotationMotor.configVoltageCompSaturation(8.0, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set voltage comp saturation!");
    TalonUtil.checkError(
        rotationMotor.configVoltageMeasurementFilter(1, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set voltage measurement filter!");
    rotationMotor.enableVoltageCompensation(true);
    TalonUtil.checkError(
        rotationMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 38.0, 38.0, 0.0), Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set supply current limit!");
    TalonUtil.checkError(
        rotationMotor.configAllowableClosedloopError(0, 0, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set allowable closed loop error!");
    // PID 0 is for position
    TalonUtil.checkError(
        rotationMotor.config_kP(0, SwerveDriveModuleConfig.Rotation.PID0_KP, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set kp!");
    TalonUtil.checkError(
        rotationMotor.config_kI(0, SwerveDriveModuleConfig.Rotation.PID0_KI, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set ki!");
    TalonUtil.checkError(
        rotationMotor.config_kD(0, SwerveDriveModuleConfig.Rotation.PID0_KD, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set kd!");
    TalonUtil.checkError(
        rotationMotor.config_kF(0, SwerveDriveModuleConfig.Rotation.PID0_KF, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set kf!");

    Timer.delay(0.05);
  }

  private synchronized void calibrateRotationEncoder() {
    rotationMotor.setSelectedSensorPosition(
        getRotationEncoderCalibrationTarget(), 0, Config.CAN_TIMEOUT_MS);
    Timer.delay(0.75);
  }

  /************************************************************************************************
   * Reset *
   ************************************************************************************************/
  public synchronized void resetModulePosition() {
    setModulePosition(modulePositionToTranslationCenter);
  }

  @Override
  public synchronized void resetSensors() {
    translationMotor.setSelectedSensorPosition(0, 0, Config.CAN_TIMEOUT_MS);
    resetModulePosition();
  }

  /************************************************************************************************
   * Function Enabler *
   ************************************************************************************************/
  public synchronized void enableTranslationInverted(boolean invert) {
    translationMotor.setInverted(SwerveDriveModuleConfig.Translation.IS_INVERT ^ invert);
  }

  public synchronized void enableRotationMotorInverted(boolean invert) {
    rotationMotor.setInverted(SwerveDriveModuleConfig.Rotation.IS_INVERT ^ invert);
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public synchronized void setIsStandardCarpetSide(boolean is_standard_carpet_side) {
    isStandardCarpetSide = is_standard_carpet_side;
  }

  private double getTranslationDisplacement() {
    return translationEncoderUnitsToInches(periodicInput.translationMotorEncoderPosition);
  }

  private double getPreviousTranslationDisplacement() {
    return translationEncoderUnitsToInches(periodicInput.previousTranslationMotorEncoderPosition);
  }

  public double getTranslationVelocity() {
    return translationEncoderVelocityToInchesPerSecond(
        periodicInput.translationMotorEncoderVelocity);
  }

  public synchronized void setTranslationVelocityTarget(double velocity_inches_per_second) {
    periodicOutput.translationMotorControlMode = ControlMode.Velocity;
    periodicOutput.translationMotorSetpoint =
        inchesPerSecondToTranslationEncoderVelocity(velocity_inches_per_second);
  }

  public synchronized void setNormalizedTranslationVelocityTarget(double normalized_velocity) {
    setTranslationVelocityTarget(
        SwerveDriveModuleConfig.Translation.MAX_CRUISE_SPEED * normalized_velocity);
  }

  public synchronized void setTranslationOpenLoop(double normalized_output) {
    periodicOutput.translationMotorControlMode = ControlMode.PercentOutput;
    periodicOutput.translationMotorSetpoint = normalized_output;
  }

  public boolean isTranslationVelocityOnTarget() {
    if (periodicOutput.translationMotorControlMode == ControlMode.Velocity) {
      return Util.epsilonEquals(
          periodicOutput.translationMotorSetpoint,
          periodicInput.translationMotorEncoderVelocity,
          inchesPerSecondToTranslationEncoderVelocity(
              SwerveDriveModuleConfig.Translation.SPEED_ON_TARGET_ERROR_TOLERANCE));
    }
    return false;
  }

  private double getRawRotationHeading() {
    return rotationEncoderUnitsToDegrees(periodicInput.rotationMotorEncoderPosition);
  }

  public Rotation2d getRobotCentricRotationHeading() {
    return Rotation2d.fromDegrees(
        getRawRotationHeading() - rotationEncoderUnitsToDegrees(rotationCalibrationOffset));
  }

  public Rotation2d getFieldCentricRotationHeading(Rotation2d robot_heading) {
    return getRobotCentricRotationHeading().rotateBy(robot_heading);
  }

  public void setRotationHeadingTarget(double heading_degrees) {
    periodicOutput.rotationMotorControlMode = ControlMode.Position;
    periodicOutput.rotationMotorSetpoint =
        degreesToRotationEncoderUnits(
            Util.boundAngleToClosestScope(
                heading_degrees + rotationEncoderUnitsToDegrees(rotationCalibrationOffset),
                getRawRotationHeading()));
  }

  public synchronized void setRotationHeadingTarget(Rotation2d heading) {
    setRotationHeadingTarget(heading.getUnboundedDegrees());
  }

  public boolean isRotationHeadingOnTarget() {
    if (periodicOutput.rotationMotorControlMode == ControlMode.Position) {
      return Util.epsilonEquals(
          periodicOutput.rotationMotorSetpoint,
          periodicInput.rotationMotorEncoderPosition,
          degreesToRotationEncoderUnits(
              SwerveDriveModuleConfig.Rotation.HEADING_ON_TARGET_ERROR_TOLERANCE));
    }
    return false;
  }

  public synchronized void setRotationOpenLoop(double normalized_output) {
    periodicOutput.rotationMotorControlMode = ControlMode.PercentOutput;
    periodicOutput.rotationMotorSetpoint = normalized_output;
  }

  public int getExternalRotationEncoderPosition() {
    return Util.conditionalInvert(
        (int) externalRotationEncoder.getPosition(),
        SwerveDriveModuleConfig.Rotation.IS_EXTERNAL_SENSOR_INVERT);
  }

  public int getRotationEncoderCalibrationTarget() {
    return Util.roundToInt(
        Util.boundToNonnegativeScope(
            Util.conditionalInvert(
                getExternalRotationEncoderPosition()
                    / SwerveDriveModuleConfig.Rotation.ENCODER_TO_EXTERNAL_ENCODER_RATIO,
                rotationMotor.getInverted()),
            SwerveDriveModuleConfig.Rotation.ENCODER_UNITS_PER_MODULE_BASE_REVOLUTION));
  }

  public Translation2d getModulePosition() {
    return modulePosition;
  }

  public Pose2d getEstimatedRobotPose() {
    return estimatedRobotPose;
  }

  public synchronized void setModulePosition(Translation2d module_position) {
    this.modulePosition = module_position;
  }

  public synchronized void setModulePositionFromRobotPose(Pose2d robot_pose) {
    setModulePosition(
        robot_pose
            .transformBy(Pose2d.fromTranslation(modulePositionToTranslationCenter))
            .getTranslation());
    estimatedRobotPose = robot_pose;
  }

  /************************************************************************************************
   * Update *
   ************************************************************************************************/
  public synchronized void updateOdometer(Rotation2d robot_heading) {
    double deltaEncoderPosition =
        getTranslationDisplacement() - getPreviousTranslationDisplacement();
    Rotation2d rotationHeading = getFieldCentricRotationHeading(robot_heading);
    double deltaTranslationX = rotationHeading.cos() * deltaEncoderPosition;
    double deltaTranslationY = rotationHeading.sin() * deltaEncoderPosition;

    if (SwerveDriveModuleConfig.ENABLE_SCRUB_FACTOR) {
      if (Math.signum(deltaTranslationX) > 0.0) {
        if (isStandardCarpetSide) {
          deltaTranslationX *= 1.0 / SwerveDriveModuleConfig.X_SCRUB_FACTOR;
        }
      } else {
        if (isStandardCarpetSide) {
          deltaTranslationX *= Math.pow(SwerveDriveModuleConfig.X_SCRUB_FACTOR, 2.0);
        }
      }
      if (Math.signum(deltaTranslationY) > 0.0) {
        if (isStandardCarpetSide) {
          deltaTranslationY *= 1.0 / SwerveDriveModuleConfig.Y_SCRUB_FACTOR;
        }
      } else {
        if (isStandardCarpetSide) {
          deltaTranslationY *= Math.pow(SwerveDriveModuleConfig.Y_SCRUB_FACTOR, 2.0);
        }
      }
    }

    Translation2d deltaTranslation = new Translation2d(deltaTranslationX, deltaTranslationY);

    modulePosition = modulePosition.translateBy(deltaTranslation);
    estimatedRobotPose =
        new Pose2d(modulePosition, robot_heading)
            .transformBy(Pose2d.fromTranslation(modulePositionToTranslationCenter.inverse()));
    synchronizePreviousTranslationEncoderPosition();
  }

  /************************************************************************************************
   * Util *
   ************************************************************************************************/
  private double translationEncoderUnitsToInches(double encoder_units) {
    return encoder_units / SwerveDriveModuleConfig.Translation.ENCODER_UNITS_PER_INCH;
  }

  private int inchesToTranslationEncoderUnits(double inches) {
    return Util.roundToInt(inches * SwerveDriveModuleConfig.Translation.ENCODER_UNITS_PER_INCH);
  }

  private double translationEncoderVelocityToInchesPerSecond(double encoder_units_per_100ms) {
    return translationEncoderUnitsToInches(encoder_units_per_100ms) * 10.0;
  }

  private int inchesPerSecondToTranslationEncoderVelocity(double inches_per_second) {
    return Util.roundToInt(inchesToTranslationEncoderUnits(inches_per_second / 10.0));
  }

  private int degreesToRotationEncoderUnits(double degrees) {
    return Util.roundToInt(degrees * SwerveDriveModuleConfig.Rotation.ENCODER_UNITS_PER_DEGREE);
  }

  private double rotationEncoderUnitsToDegrees(double encoder_units) {
    return encoder_units / SwerveDriveModuleConfig.Rotation.ENCODER_UNITS_PER_DEGREE;
  }

  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  @Override
  public synchronized void disable() {
    setTranslationOpenLoop(0.0);
    setRotationOpenLoop(0.0);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  @Override
  public void logToSmartDashboard() {
    SmartDashboard.putNumber(
        moduleName + "rotation calibration position offset", getRotationEncoderCalibrationTarget());
  }

  private boolean testTranslationMotorCalibration() {
    return translationMotor.getInverted() == SwerveDriveModuleConfig.Translation.IS_INVERT
        && Util.epsilonEquals(
            translationMotor.getSelectedSensorPosition(),
            0.0,
            inchesToTranslationEncoderUnits(
                SwerveDriveModuleConfig.Translation.DISTANCE_ON_TARGET_ERROR_TOLERANCE));
  }

  private boolean testRotationMotorCalibration() {
    return rotationMotor.getInverted() == SwerveDriveModuleConfig.Rotation.IS_INVERT
        && Util.epsilonEquals(
            rotationMotor.getSelectedSensorPosition(),
            getRotationEncoderCalibrationTarget(),
            degreesToRotationEncoderUnits(
                SwerveDriveModuleConfig.Rotation.HEADING_ON_TARGET_ERROR_TOLERANCE));
  }

  @Override
  public boolean selfTest() {
    return testTranslationMotorCalibration() && testRotationMotorCalibration();
  }
}
