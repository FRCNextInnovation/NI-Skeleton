package com.nextinnovation.lib.drivers;

import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionResult {
  public boolean hasTarget;
  public double timestamp;
  public double latency;
  public Rotation2d targetHeading;
  public Rotation2d targetElevation;
  public Rotation2d targetSkew;
  public double targetDistance;
  public Translation2d targetOrientation;

  public VisionResult() {
    this(false, -1.0, 0.0, Rotation2d.identity(), Rotation2d.identity(), Rotation2d.identity());
  }

  public VisionResult(
      boolean has_target,
      double timestamp,
      double latency,
      Rotation2d target_heading,
      Rotation2d target_elevation,
      Rotation2d target_skew) {
    hasTarget = has_target;
    this.timestamp = timestamp;
    this.latency = latency;
    targetHeading = target_heading;
    targetElevation = target_elevation;
    targetSkew = target_skew;
    targetDistance = 0.0;
    targetOrientation = Translation2d.identity();
  }

  public static VisionResult fromPhotonResult(PhotonPipelineResult photon_result) {
    if (photon_result.hasTargets()) {
      var bestTarget = photon_result.getBestTarget();
      return new VisionResult(
          true,
          photon_result.getTimestampSeconds(),
          photon_result.getLatencyMillis() / 1000.0,
          Rotation2d.fromDegrees(-bestTarget.getYaw()),
          Rotation2d.fromDegrees(bestTarget.getPitch()),
          Rotation2d.fromDegrees(bestTarget.getSkew()));
    } else {
      return new VisionResult();
    }
  }
}
