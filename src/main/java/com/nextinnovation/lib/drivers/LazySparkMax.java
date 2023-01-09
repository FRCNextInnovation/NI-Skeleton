package com.nextinnovation.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class LazySparkMax extends CANSparkMax {
  protected ControlType previousControlType = null;
  protected double previousSetpoint = Double.NaN;

  protected CANSparkMax leader = null;

  public LazySparkMax(CanId id) {
    super(id.getNum(), MotorType.kBrushless);
  }

  @Override
  public REVLibError follow(CANSparkMax leader) {
    this.leader = leader;
    return super.follow(leader);
  }

  public void set(ControlType type, double setPoint) {
    if (setPoint != previousSetpoint || type != previousControlType) {
      previousSetpoint = setPoint;
      previousControlType = type;
      super.getPIDController().setReference(setPoint, type);
    }
  }
}
