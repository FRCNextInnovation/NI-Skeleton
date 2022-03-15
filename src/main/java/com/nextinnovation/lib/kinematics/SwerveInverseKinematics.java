package com.nextinnovation.lib.kinematics;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class SwerveInverseKinematics {
  private final int moduleCount;
  private final List<Translation2d> modulePositionsRelativeToDriveCenter;
  private List<Translation2d> modulePositionsRelativeToRotationCenter;
  private List<Translation2d> moduleRotationVectors;

  public SwerveInverseKinematics(
      int moduleCount, List<Translation2d> modulePositionsRelativeToDriveCenter) {
    this.moduleCount = moduleCount;
    this.modulePositionsRelativeToDriveCenter = modulePositionsRelativeToDriveCenter;
    modulePositionsRelativeToRotationCenter = modulePositionsRelativeToDriveCenter;
    resetCenterOfRotation();
  }

  public void setCenterOfRotation(Translation2d centerOfRotation) {
    modulePositionsRelativeToRotationCenter = new ArrayList<>(moduleCount);
    for (Translation2d position : modulePositionsRelativeToDriveCenter) {
      modulePositionsRelativeToRotationCenter.add(position.translateBy(centerOfRotation.inverse()));
    }
    updateModuleRotationVectors();
  }

  public void resetCenterOfRotation() {
    setCenterOfRotation(Translation2d.identity());
  }

  private void updateModuleRotationVectors() {
    moduleRotationVectors = new ArrayList<>(moduleCount);
    for (Translation2d position : modulePositionsRelativeToRotationCenter) {
      moduleRotationVectors.add(position.rotateBy(Rotation2d.fromDegrees(90.0)));
    }
  }

  public List<Translation2d> calculateModuleVelocities(
      Translation2d translationVector,
      double rotationMagnitude,
      Pose2d robotPose,
      boolean isFieldCentric) {
    if (isFieldCentric) {
      translationVector = translationVector.rotateBy(robotPose.getRotation().inverse());
    }
    List<Translation2d> moduleDriveVectors = new ArrayList<>(moduleCount);
    for (Translation2d rotationVector : moduleRotationVectors) {
      moduleDriveVectors.add(
          translationVector.translateBy(rotationVector.scale(rotationMagnitude)));
    }
    return moduleDriveVectors;
  }

  public List<Translation2d> calculateNormalizedModuleVelocities(
      Translation2d translationVector,
      double rotationMagnitude,
      Pose2d robotPose,
      boolean isFieldCentric) {
    return normalizeVectors(
        calculateModuleVelocities(translationVector, rotationMagnitude, robotPose, isFieldCentric),
        1.0);
  }

  public List<Rotation2d> calculateModuleRotationHeadingsAlignedToRotationVectors() {
    return calculateModuleRotationHeadingsAlignedToRotationVectors(new Rotation2d());
  }

  public List<Rotation2d> calculateModuleRotationHeadingsAlignedToRotationVectors(
      Rotation2d rotationOffset) {
    List<Rotation2d> rotationHeadings = new ArrayList<>(moduleRotationVectors.size());
    for (Translation2d rotationVector : moduleRotationVectors) {
      rotationHeadings.add(rotationVector.direction().rotateBy(rotationOffset));
    }
    return rotationHeadings;
  }

  private List<Translation2d> normalizeVectors(
      List<Translation2d> vectors, double referenceMagnitude) {
    List<Translation2d> newVectors = new ArrayList<>(vectors.size());
    for (Translation2d vector : vectors) {
      referenceMagnitude = Math.max(vector.norm(), referenceMagnitude);
    }
    for (Translation2d vector : vectors) {
      newVectors.add(vector.scale(1.0 / referenceMagnitude));
    }
    return newVectors;
  }
}
