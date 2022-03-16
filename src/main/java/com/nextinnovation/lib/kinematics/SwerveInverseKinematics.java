package com.nextinnovation.lib.kinematics;

import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class SwerveInverseKinematics {
  private final int moduleCount;
  private final List<Translation2d> modulePositionsRelativeToDriveCenter;
  private List<Translation2d> modulePositionsRelativeToRotationCenter;
  private List<Rotation2d> moduleRotationVectors;

  public SwerveInverseKinematics(
      int module_count, List<Translation2d> module_positions_relative_to_drive_center) {
    moduleCount = module_count;
    modulePositionsRelativeToDriveCenter = module_positions_relative_to_drive_center;
    modulePositionsRelativeToRotationCenter = modulePositionsRelativeToDriveCenter;
    resetCenterOfRotation();
  }

  public void setCenterOfRotation(Translation2d center_of_rotation) {
    modulePositionsRelativeToRotationCenter = new ArrayList<>(moduleCount);
    for (Translation2d position : modulePositionsRelativeToDriveCenter) {
      modulePositionsRelativeToRotationCenter.add(
          position.translateBy(center_of_rotation.inverse()));
    }
    moduleRotationVectors = new ArrayList<>(moduleCount);
    for (Translation2d position : modulePositionsRelativeToRotationCenter) {
      moduleRotationVectors.add(position.rotateBy(Rotation2d.fromDegrees(90.0)).direction());
    }
  }

  public void resetCenterOfRotation() {
    setCenterOfRotation(Translation2d.identity());
  }

  public List<Translation2d> calculateNormalizedModuleVelocities(
      Translation2d translation_vector,
      double rotation_magnitude,
      Rotation2d field_centric_robot_heading) {
    translation_vector = translation_vector.rotateBy(field_centric_robot_heading.inverse());
    List<Translation2d> moduleDriveVectors = new ArrayList<>(moduleCount);
    double maxCalculatedModuleVelocity = 1.0;
    for (Rotation2d rotationVector : moduleRotationVectors) {
      Translation2d driveVector =
          translation_vector.translateBy(
              Translation2d.fromPolar(rotationVector, rotation_magnitude));
      double translationVelocity = driveVector.norm();

      if (translationVelocity > maxCalculatedModuleVelocity) {
        maxCalculatedModuleVelocity = translationVelocity;
      }

      moduleDriveVectors.add(driveVector);
    }

    double velocityConstraintScalar = 1.0 / maxCalculatedModuleVelocity;
    for (int i = 0; i < moduleDriveVectors.size(); i++) {
      Translation2d driveVector = moduleDriveVectors.get(i);
      moduleDriveVectors.set(
          i,
          Translation2d.fromPolar(
              driveVector.direction(), driveVector.norm() * velocityConstraintScalar));
    }

    return moduleDriveVectors;
  }
}
