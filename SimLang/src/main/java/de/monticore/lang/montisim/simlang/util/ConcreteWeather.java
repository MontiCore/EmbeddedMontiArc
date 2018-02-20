package de.monticore.lang.montisim.simlang.util;

import java.util.ArrayList;
import java.util.Optional;

public class ConcreteWeather {


  private Optional<NumberUnit> temperature;
  private Optional<SimLangEnums.CloudingTypes> clouding;
  private Optional<NumberUnit> sight;
  private Optional<SimLangEnums.PrecipitationTypes> precipitationtype;
  private Optional<NumberUnit> humidity;
  private Optional<NumberUnit> pressure;
  private Optional<NumberUnit> windstrength;
  private Optional<NumberUnit> winddirection;
  private Optional<NumberUnit> precipitationamount;
  private Optional<ArrayList<WeatherPhenomenaInstance>> weatherPhenomena;
  private Optional<SimLangEnums.OpticalPhenomenas> opticalPhenomena;
  private Optional<SimLangEnums.ArtificialPhenomena> artificialPhenomena;

  public ConcreteWeather(NumberUnit temperature, SimLangEnums.CloudingTypes clouding, NumberUnit sight,
                         SimLangEnums.PrecipitationTypes precipitationtype, NumberUnit humidity,
                         NumberUnit pressure, NumberUnit windstrength, NumberUnit winddirection,
                         NumberUnit precipitationamount,
                         ArrayList<WeatherPhenomenaInstance> weatherPhenomena,
                         SimLangEnums.OpticalPhenomenas opticalPhenomena, SimLangEnums.ArtificialPhenomena artificialPhenomena) {

    this.temperature = temperature != null ? Optional.of(temperature) : Optional.empty();
    this.clouding = clouding != null ? Optional.of(clouding) : Optional.empty();
    this.sight = sight != null ? Optional.of(sight) : Optional.empty();
    this.precipitationtype = precipitationtype != null ? Optional.of(precipitationtype) : Optional.empty();
    this.humidity = humidity != null ? Optional.of(humidity) : Optional.empty();
    this.pressure = pressure != null ? Optional.of(pressure) : Optional.empty();
    this.windstrength = windstrength != null ? Optional.of(windstrength) : Optional.empty();
    this.winddirection = winddirection != null ? Optional.of(winddirection) : Optional.empty();
    this.precipitationamount = precipitationamount != null ? Optional.of(precipitationamount) : Optional.empty();
    this.weatherPhenomena = weatherPhenomena != null ? Optional.of(weatherPhenomena) : Optional.empty();
    this.opticalPhenomena = opticalPhenomena != null ? Optional.of(opticalPhenomena) : Optional.empty();
    this.artificialPhenomena = artificialPhenomena != null ? Optional.of(artificialPhenomena) : Optional.empty();
  }

  public Optional<NumberUnit> getTemperature() {
    return temperature;
  }

  public Optional<SimLangEnums.CloudingTypes> getClouding() {
    return clouding;
  }

  public Optional<NumberUnit> getSight() {
    return sight;
  }

  public Optional<SimLangEnums.PrecipitationTypes> getPrecipitationtype() {
    return precipitationtype;
  }

  public Optional<NumberUnit> getHumidity() {
    return humidity;
  }

  public Optional<NumberUnit> getPressure() {
    return pressure;
  }

  public Optional<NumberUnit> getWindstrength() {
    return windstrength;
  }

  public Optional<NumberUnit> getWinddirection() {
    return winddirection;
  }

  public Optional<NumberUnit> getPrecipitationamount() {
    return precipitationamount;
  }

  public Optional<ArrayList<WeatherPhenomenaInstance>> getWeatherPhenomena() {
    return weatherPhenomena;
  }

  public Optional<SimLangEnums.OpticalPhenomenas> getOpticalPhenomena() {
    return opticalPhenomena;
  }

  public Optional<SimLangEnums.ArtificialPhenomena> getArtificialPhenomena() {
    return artificialPhenomena;
  }
}
