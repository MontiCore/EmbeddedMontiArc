/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

public class Channel {
  SimLangEnums.ChannelTypes type;
  String name;
  AlternativeInput transferRate;
  AlternativeInput latency;
  AlternativeInput outage;
  Area area;

  public Channel(SimLangEnums.ChannelTypes type, String name, AlternativeInput transferRate, AlternativeInput latency, AlternativeInput outage, Area area) {
    this.type = type;
    this.name = name;
    this.transferRate = transferRate;
    this.latency = latency;
    this.outage = outage;
    this.area = area;
  }

  public SimLangEnums.ChannelTypes getType() {
    return type;
  }

  public String getName() {
    return name;
  }

  public AlternativeInput getTransferRate() {
    return transferRate;
  }

  public AlternativeInput getLatency() {
    return latency;
  }

  public AlternativeInput getOutage() {
    return outage;
  }

  public Area getArea() {return this.area;}
}
