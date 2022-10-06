package com.nextinnovation.lib.log;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.geometry.Rotation2d;

import com.nextinnovation.lib.utils.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldViewer {
  private final Field2d robotPose;

  private final double poseXOffset;
  private final double poseYOffset;

  public FieldViewer() {
    this(0.0, 0.0);
  }

  public FieldViewer(double x_start_position, double y_start_position) {
    robotPose = new Field2d();

    poseXOffset = x_start_position;
    poseYOffset = y_start_position;

    SmartDashboard.putData(robotPose);
  }

  public void setRobotPose(Pose2d robot_pose) {
    Translation2d translation = robot_pose.getTranslation();
    Rotation2d rotation = robot_pose.getRotation();

    robotPose.setRobotPose(
        new edu.wpi.first.math.geometry.Pose2d(
            new edu.wpi.first.math.geometry.Translation2d(
                Units.inches_to_meters(translation.x() - poseXOffset),
                Units.inches_to_meters(translation.y() - poseYOffset)),
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(rotation.getDegrees())));
  }
}
