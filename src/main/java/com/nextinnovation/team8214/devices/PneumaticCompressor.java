package com.nextinnovation.team8214.devices;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticCompressor {
  private static PneumaticCompressor instance = null;

  public static synchronized PneumaticCompressor getInstance() {
    if (instance == null) {
      instance = new PneumaticCompressor();
    }
    return instance;
  }

  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public void enable() {
    compressor.enableDigital();
  }

  public void disable() {
    compressor.disable();
  }

  public boolean isEnabled() {
    return compressor.enabled();
  }

  public boolean pressureOnTarget() {
    return compressor.getPressureSwitchValue();
  }
}
