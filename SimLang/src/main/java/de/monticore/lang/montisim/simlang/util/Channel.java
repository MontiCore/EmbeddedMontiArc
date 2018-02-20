package de.monticore.lang.montisim.simlang.util;

public class Channel {
  String type;
  String name;
  NumberUnit transferrate;
  NumberUnit latency;
  Float outage;
  boolean global = false;

  public Channel(String type, String name, NumberUnit transferrate, NumberUnit latency, Float outage) {
    this.type = type;
    this.name = name;
    this.transferrate = transferrate;
    this.latency = latency;
    this.outage = outage;
  }

  public Channel(String type, String name, NumberUnit transferrate, NumberUnit latency, Float outage, boolean global) {
    this.type = type;
    this.name = name;
    this.transferrate = transferrate;
    this.latency = latency;
    this.outage = outage;
    this.global = global;
  }

  public String getType() {
    return type;
  }

  public String getName() {
    return name;
  }

  public NumberUnit getTransferrate() {
    return transferrate;
  }

  public NumberUnit getLatency() {
    return latency;
  }

  public Float getOutage() {
    return outage;
  }

  public boolean isGlobal() {
    return global;
  }
}