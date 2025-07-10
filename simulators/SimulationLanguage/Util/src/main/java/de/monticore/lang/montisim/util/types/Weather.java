/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import java.util.Optional;

public class Weather {
  private Optional<FixedWeather> fixedWeather = Optional.empty();
  private Optional<SequenceWeather> sequenceWeather = Optional.empty();
  private Optional<RandomWeather> randomWeather = Optional.empty();

  public Weather(FixedWeather fixedWeather) {
    this.fixedWeather = Optional.of(fixedWeather);
  }
  public Weather(SequenceWeather sequenceWeather) {
    this.sequenceWeather = Optional.of(sequenceWeather);
  }
  public Weather(RandomWeather randomWeather) {
    this.randomWeather = Optional.of(randomWeather);
  }

  public Optional<FixedWeather> getFixedWeather() {
    return fixedWeather;
  }

  public Optional<SequenceWeather> getSequenceWeather() {
    return sequenceWeather;
  }

  public Optional<RandomWeather> getRandomWeather() {
    return randomWeather;
  }
}
