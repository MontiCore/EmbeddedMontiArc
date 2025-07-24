/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import java.util.Optional;

public class Time {
  private int hours;
  private int minutes;
  private Optional<Integer> seconds = Optional.empty();
  private Optional<Integer> milliseconds = Optional.empty();

  public Time(int hours, int minutes) {
    this.hours = hours;
    this.minutes = minutes;
  }
  public Time(int hours, int minutes, Integer seconds, Integer milliseconds) {
    this.hours = hours;
    this.minutes = minutes;
    if(seconds != null) {
      this.seconds = Optional.of(seconds);
    }
    if(milliseconds != null) {
      this.milliseconds = Optional.of(milliseconds);
    }
  }

  public int getHours() {
    return hours;
  }

  public int getMinutes() {
    return minutes;
  }

  public Optional<Integer> getSeconds() {
    return seconds;
  }

  public Optional<Integer> getMilliseconds() {
    return milliseconds;
  }
}
