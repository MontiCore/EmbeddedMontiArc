package de.monticore.lang.montisim.simlang.util;

import java.util.Optional;

public class Time {
  Float hours;
  Float minutes;
  Optional<Float> seconds = Optional.empty();
  Optional<Float> milliseconds = Optional.empty();

  public Time(String hours, String minutes) {
    this.hours = Float.parseFloat(hours);
    this.minutes = Float.parseFloat(minutes);
  }
  public Time(String hours, String minutes, Optional<String> seconds, Optional<String> milliseconds) {
    this.hours = Float.parseFloat(hours);
    this.minutes = Float.parseFloat(minutes);
    if(seconds.isPresent()) { this.seconds = Optional.of(Float.parseFloat(seconds.get())); }
    if(milliseconds.isPresent()) {this.milliseconds = Optional.of(Float.parseFloat(milliseconds.get())); }
  }

  public Float getHours() {
    return hours;
  }

  public Float getMinutes() {
    return minutes;
  }

  public Optional getSeconds() {
    return seconds;
  }

  public Optional getMilliseconds() {
    return milliseconds;
  }
}