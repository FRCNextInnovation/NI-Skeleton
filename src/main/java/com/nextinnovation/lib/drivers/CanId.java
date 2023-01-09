package com.nextinnovation.lib.drivers;

import java.util.Objects;

public class CanId {
  private final int num;
  private final String bus;

  public CanId(int num, String bus) {
    this.num = num;
    this.bus = bus;
  }

  public CanId(int num) {
    this(num, "");
  }

  public int getNum() {
    return num;
  }

  public String getBus() {
    return bus;
  }

  public boolean equals(CanId other) {
    return other.num == num && Objects.equals(other.bus, bus);
  }
}
