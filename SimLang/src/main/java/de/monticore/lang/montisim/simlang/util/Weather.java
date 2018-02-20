package de.monticore.lang.montisim.simlang.util;

import java.util.ArrayList;

public class Weather {

  private SimLangEnums.WeatherTypes type;
  private ArrayList<ConcreteWeather> weatherObjects;
  private ArrayList<NumberUnit> durations;
  private String automatonPlaceholder;

  //Call this constructor for fixed weather
  public Weather(ConcreteWeather fixedWeather) {
    this.type = SimLangEnums.WeatherTypes.FIXED;
    this.weatherObjects = new ArrayList<ConcreteWeather>();
    this.weatherObjects.add(fixedWeather);
    this.durations = null; //todo: think this over
  }

  //Call this constructor for sequencial weather
  public Weather(ArrayList<ConcreteWeather> weatherList, ArrayList<NumberUnit> durationList) {
    this.type = SimLangEnums.WeatherTypes.SEQUENCE;
    this.weatherObjects = weatherList;
    this.durations = durationList;
  }

  //Call this constructor for random weather
  public Weather(NumberUnit duration) {
    this.type = SimLangEnums.WeatherTypes.RANDOM;
    this.durations = new ArrayList<NumberUnit>();
    this.durations.add(duration);
  }

  //Call this constructor for forecast weather
  public Weather(ConcreteWeather initialWeather, NumberUnit duration, String autamaton) {
    this.type = SimLangEnums.WeatherTypes.FORECAST;
    this.durations = new ArrayList<NumberUnit>();
    this.durations.add(duration);
    this.automatonPlaceholder = autamaton;
  }

  public SimLangEnums.WeatherTypes getType() {
    return type;
  }

  public ArrayList<ConcreteWeather> getWeatherObjects() {
    return weatherObjects;
  }

  public ArrayList<NumberUnit> getDurations() {
    return durations;
  }

  public String getAutomatonPlaceholder() {
    return automatonPlaceholder;
  }
}
