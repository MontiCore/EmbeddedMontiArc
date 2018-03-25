package de.monticore.lang.montisim.simlang.util;


import de.monticore.lang.montisim.weather.cocos.NumberUnit;

public class Range {
  NumberUnit start;
  NumberUnit step;
  NumberUnit end;

  public Range(NumberUnit start, NumberUnit step, NumberUnit end) {
    this.start = start;
    this.step = step;
    this.end = end;
  }

  public NumberUnit getStart() {
    return start;
  }

  public NumberUnit getStep() {
    return step;
  }

  public NumberUnit getEnd() {
    return end;
  }
}