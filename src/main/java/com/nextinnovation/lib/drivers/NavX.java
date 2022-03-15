package com.nextinnovation.lib.drivers;

import com.kauailabs.navx.frc.AHRS;

public class NavX {
  private final AHRS ahrs;

  public NavX() {
    ahrs = new AHRS();
  }

  public boolean isCalibrating() {
    return ahrs.isCalibrating();
  }

  public boolean isConnected() {
    return ahrs.isConnected();
  }

  public double getFusedYaw() {
    return -ahrs.getFusedHeading();
  }

  public double getRawYaw() {
    return ahrs.getYaw();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  public double getRoll() {
    return ahrs.getRoll();
  }

  public double getGyroX() {
    return ahrs.getRawGyroX();
  }

  public double getGyroY() {
    return ahrs.getRawGyroY();
  }

  public double getGyroZ() {
    return ahrs.getRawGyroZ();
  }

  public double getVelocityX() {
    return ahrs.getVelocityX();
  }

  public double getVelocityY() {
    return ahrs.getVelocityY();
  }

  public double getVelocityZ() {
    return ahrs.getVelocityZ();
  }

  public double getTempC() {
    return ahrs.getTempC();
  }

  public synchronized void setYawToZero() {
    ahrs.zeroYaw();
  }

  public synchronized void resetVelocityIntergration() {
    ahrs.resetDisplacement();
  }
}
