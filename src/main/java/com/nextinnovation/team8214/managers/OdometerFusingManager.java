package com.nextinnovation.team8214.managers;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.log.FieldViewer;
import com.nextinnovation.team8214.Field;
import com.nextinnovation.team8214.subsystems.swerve.SwerveConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

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
    var initModulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < initModulePositions.length; i++) {
      initModulePositions[i] = new SwerveModulePosition();
    }

    fieldViewer = new FieldViewer(Field.X_MIN, Field.Y_MIN);
    estimator =
        new SwerveDrivePoseEstimator(
            SwerveConfig.WPILIB_SWERVE_KINEMATICS,
            new edu.wpi.first.math.geometry.Rotation2d(),
            initModulePositions,
            new edu.wpi.first.math.geometry.Pose2d(),
            VecBuilder.fill(0.02, 0.02, 0.01),
            VecBuilder.fill(0.1, 0.1, 0.01));
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
  public void setPose(Pose2d pose, Rotation2d heading, SwerveModulePosition... modulePositions) {
    estimator.resetPosition(heading.toWpilibRotation2d(), modulePositions, pose.toWpilibPose2d());
  }

  public Pose2d getLatestFieldCentricRobotPose() {
    return Pose2d.fromWpilibPose2d(estimator.getEstimatedPosition());
  }

  /************************************************************************************************
   * Update *
   ************************************************************************************************/
  public synchronized void updateWO(Rotation2d heading, SwerveModulePosition... modulePositions) {
    estimator.update(heading.toWpilibRotation2d(), modulePositions);
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
