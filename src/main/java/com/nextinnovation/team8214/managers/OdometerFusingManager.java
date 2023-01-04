package com.nextinnovation.team8214.managers;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.log.FieldViewer;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Field;
import com.nextinnovation.team8214.subsystems.swerve.SwerveConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class OdometerFusingManager {
  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static OdometerFusingManager instance = null;

  public static synchronized OdometerFusingManager getInstance() {
    if (instance == null) {
      instance = new OdometerFusingManager();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private Boolean isVOEnabled = true;

  private final SwerveDrivePoseEstimator estimator;
  private final FieldViewer fieldViewer;

  private OdometerFusingManager() {
    fieldViewer = new FieldViewer(Field.X_MIN, Field.Y_MIN);
    estimator =
        new SwerveDrivePoseEstimator(
            new edu.wpi.first.math.geometry.Rotation2d(),
            new edu.wpi.first.math.geometry.Pose2d(),
            SwerveConfig.WPILIB_SWERVE_KINEMATICS,
            VecBuilder.fill(0.02, 0.02, 0.01),
            VecBuilder.fill(0.01),
            VecBuilder.fill(0.1, 0.1, 0.01),
            Config.LOOPER_CONTROL_PERIOD_SEC);
  }

  /************************************************************************************************
   * Enable & Disable *
   ************************************************************************************************/
  public synchronized void enableVO() {
    isVOEnabled = true;
  }

  public synchronized void disableVO() {
    isVOEnabled = false;
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public void setPose(Pose2d pose, Rotation2d heading) {
    estimator.resetPosition(pose.toWpilibPose2d(), heading.toWpilibRotation2d());
  }

  public Pose2d getLatestFieldCentricRobotPose() {
    return Pose2d.fromWpilibPose2d(estimator.getEstimatedPosition());
  }

  /************************************************************************************************
   * Update *
   ************************************************************************************************/
  public synchronized void updateWO(Rotation2d heading, SwerveModuleState... moduleStates) {
    estimator.update(heading.toWpilibRotation2d(), moduleStates);
  }

  public synchronized void updateVO(double timestamp, Pose2d vo_estimated_pose) {
    if (isVOEnabled) {
      estimator.addVisionMeasurement(vo_estimated_pose.toWpilibPose2d(), timestamp);
    }
  }

  /************************************************************************************************
   * Log *
   ************************************************************************************************/
  public void logToSmartDashBoard() {
    fieldViewer.setRobotPose(getLatestFieldCentricRobotPose());
  }
}
