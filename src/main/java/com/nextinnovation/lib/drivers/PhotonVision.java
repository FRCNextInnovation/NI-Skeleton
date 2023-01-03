package com.nextinnovation.lib.drivers;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision {
  private final PhotonCamera camera;
  private PhotonPipelineResult latest_result;

  public PhotonVision(String camera_name) {
    camera = new PhotonCamera(camera_name);
    latest_result = new PhotonPipelineResult();
  }

  public void update() {
    latest_result = camera.getLatestResult();
  }

  public PhotonPipelineResult getRawVisionResult() {
    return latest_result;
  }

  public VisionResult getVisionResult() {
    return VisionResult.fromPhotonResult(latest_result);
  }

  public void setLed(VisionLEDMode led_mode) {
    camera.setLED(led_mode);
  }
}
