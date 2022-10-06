package com.nextinnovation.team8214.managers;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.log.FieldViewer;
import com.nextinnovation.lib.utils.InterpolatingDouble;
import com.nextinnovation.lib.utils.InterpolatingTreeMap;
import com.nextinnovation.lib.utils.Util;
import com.nextinnovation.team8214.Field;

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
  private static final int ODOMETRY_SAMPLE_SIZE = 80;

  private Boolean isVOEnabled = true;

  // ! Pose: Delta translation with heading.
  private InterpolatingTreeMap<InterpolatingDouble, Pose2d> chassisWODeltaTranslationMap;

  private InterpolatingTreeMap<InterpolatingDouble, Pose2d> fusedOdometry;

  private final FieldViewer fieldViewer;

  private OdometerFusingManager() {
    fieldViewer = new FieldViewer(Field.X_MIN, Field.Y_MIN);

    reset(0.0, Pose2d.identity());
  }

  /************************************************************************************************
   * Reset *
   ************************************************************************************************/
  public synchronized void reset(double start_time, Pose2d init_robot_pose) {
    fusedOdometry = new InterpolatingTreeMap<>(ODOMETRY_SAMPLE_SIZE);
    fusedOdometry.put(new InterpolatingDouble(start_time), init_robot_pose);

    chassisWODeltaTranslationMap = new InterpolatingTreeMap<>(ODOMETRY_SAMPLE_SIZE);
    chassisWODeltaTranslationMap.put(
        new InterpolatingDouble(start_time),
        new Pose2d(new Translation2d(), init_robot_pose.getRotation()));
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
  public Pose2d getLatestFieldCentricRobotPose() {
    return fusedOdometry.lastEntry().getValue();
  }

  public Pose2d getFieldCentricRobotPoseByTimestamp(double timestamp) {
    return fusedOdometry.getInterpolated(new InterpolatingDouble(timestamp));
  }

  // ! Real WO are kept in swerve.
  public synchronized void addChassisWODeltaTranslationMap(
      double timestamp, Translation2d delta_position, Rotation2d currant_heading) {
    // ! All pose & translation here is field centric.
    chassisWODeltaTranslationMap.put(
        new InterpolatingDouble(timestamp), new Pose2d(delta_position, currant_heading));

    Translation2d updatedTranslation =
        fusedOdometry.lastEntry().getValue().getTranslation().translateBy(delta_position);

    fusedOdometry.put(
        new InterpolatingDouble(timestamp), new Pose2d(updatedTranslation, currant_heading));
  }

  public synchronized void addVOKeyFrame(double timestamp, Pose2d VO_estimated_pose) {
    if (isVOEnabled) {
      fusedOdometry.put(new InterpolatingDouble(timestamp), VO_estimated_pose);
      rollingOptimization(timestamp);
    }
  }

  private synchronized void rollingOptimization(double latest_vo_sample_timestamp) {
    // ! Measure new robot pose again by latest key frame from VO.
    InterpolatingDouble startTimestamp = new InterpolatingDouble(latest_vo_sample_timestamp);
    Pose2d startPose = fusedOdometry.getInterpolated(startTimestamp); // VO_estimated_pose

    if (chassisWODeltaTranslationMap.higherKey(startTimestamp) == null) {
      return;
    }

    while (!(Util.epsilonEquals(
        chassisWODeltaTranslationMap.ceilingKey(startTimestamp).value,
        chassisWODeltaTranslationMap.lastEntry().getKey().value))) {
      InterpolatingDouble endTimestamp = chassisWODeltaTranslationMap.higherKey(startTimestamp);
      Translation2d endTranslation =
          chassisWODeltaTranslationMap.getInterpolated(endTimestamp).getTranslation();

      Pose2d optimizedPose = startPose.translateBy(endTranslation);

      fusedOdometry.put(endTimestamp, optimizedPose);
      startPose = optimizedPose;
      startTimestamp = endTimestamp;
    }
  }

  /************************************************************************************************
   * Log *
   ************************************************************************************************/
  public void logToSmartDashBoard() {
    fieldViewer.setRobotPose(getLatestFieldCentricRobotPose());
  }
}
