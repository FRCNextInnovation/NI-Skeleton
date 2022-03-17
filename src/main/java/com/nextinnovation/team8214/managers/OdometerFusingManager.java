package com.nextinnovation.team8214.managers;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.utils.InterpolatingDouble;
import com.nextinnovation.lib.utils.InterpolatingTreeMap;

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

  // ! Pose: Delta translation with heading.
  private InterpolatingTreeMap<InterpolatingDouble, Pose2d> chassisWODeltaTranslationMap;

  private InterpolatingTreeMap<InterpolatingDouble, Pose2d> fusedOdometry;

  private OdometerFusingManager() {
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
   * Getter & Setter *
   ************************************************************************************************/
  public Pose2d getLatestFieldCentricRobotPose() {
    return fusedOdometry.lastEntry().getValue();
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
    fusedOdometry.put(new InterpolatingDouble(timestamp), VO_estimated_pose);
    rollingOptimization(timestamp);
  }

  private synchronized void rollingOptimization(double latest_vo_sample_timestamp) {
    // ! Measure new robot pose again by latest key frame from VO.
    InterpolatingDouble startTimestamp = new InterpolatingDouble(latest_vo_sample_timestamp);
    Pose2d startPose = fusedOdometry.getInterpolated(startTimestamp); // VO_estimated_pose

    while (!(chassisWODeltaTranslationMap.ceilingKey(startTimestamp) == startTimestamp
        || chassisWODeltaTranslationMap.ceilingKey(startTimestamp) == null)) {
      InterpolatingDouble endTimestamp = chassisWODeltaTranslationMap.ceilingKey(startTimestamp);
      Translation2d endTranslation =
          chassisWODeltaTranslationMap.getInterpolated(endTimestamp).getTranslation();

      InterpolatingDouble lastTimeStamp = chassisWODeltaTranslationMap.floorKey(startTimestamp);
      Translation2d deltaTranslation;

      if (lastTimeStamp == null || lastTimeStamp == startTimestamp) {
        deltaTranslation = endTranslation;
      } else {
        deltaTranslation =
            endTranslation.scale(
                ((endTimestamp.value - startTimestamp.value)
                    / (endTimestamp.value - lastTimeStamp.value)));
      }

      Pose2d optimizedPose = startPose.translateBy(deltaTranslation);
      fusedOdometry.put(endTimestamp, optimizedPose);
      startPose = optimizedPose;
      startTimestamp = endTimestamp;
    }
  }
}
