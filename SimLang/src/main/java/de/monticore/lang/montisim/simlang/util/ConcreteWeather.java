package de.monticore.lang.montisim.simlang.util;

import de.monticore.lang.montisim.weather.cocos.NumberUnit;

import java.util.ArrayList;
import java.util.Optional;

public class ConcreteWeather {

  private Optional<AlternativeInput> temperature;
  private Optional<SimLangEnums.CloudingTypes> clouding;
  private Optional<Sight> sight;
  private Optional<SimLangEnums.PrecipitationTypes> precipitationType;
  private Optional<AlternativeInput> humidity;
  private Optional<AlternativeInput> pressure;
  private Optional<AlternativeInput> windStrength;
  private Optional<AlternativeInput> windDirection;
  private Optional<AlternativeInput> precipitationAmount;
  private Optional<ArrayList<WeatherPhenomenaInstance>> weatherPhenomena;
  private Optional<ArrayList<SimLangEnums.OpticalPhenomenas>> opticalPhenomena;
  private Optional<ArrayList<SimLangEnums.ArtificialPhenomena>> artificialPhenomena;

  public ConcreteWeather(AlternativeInput temperature, SimLangEnums.CloudingTypes clouding, Sight sight,
                         SimLangEnums.PrecipitationTypes precipitationtype, AlternativeInput humidity,
                         AlternativeInput pressure, AlternativeInput windstrength, AlternativeInput winddirection,
                         AlternativeInput precipitationamount,
                         ArrayList<WeatherPhenomenaInstance> weatherPhenomena,
                         ArrayList<SimLangEnums.OpticalPhenomenas> opticalPhenomena, ArrayList<SimLangEnums.ArtificialPhenomena> artificialPhenomena) {

    this.temperature = temperature != null ? Optional.of(temperature) : Optional.empty();
    this.clouding = clouding != null ? Optional.of(clouding) : Optional.empty();
    this.sight = sight != null ? Optional.of(sight) : Optional.empty();
    this.precipitationType = precipitationtype != null ? Optional.of(precipitationtype) : Optional.empty();
    this.humidity = humidity != null ? Optional.of(humidity) : Optional.empty();
    this.pressure = pressure != null ? Optional.of(pressure) : Optional.empty();
    this.windStrength = windstrength != null ? Optional.of(windstrength) : Optional.empty();
    this.windDirection = winddirection != null ? Optional.of(winddirection) : Optional.empty();
    this.precipitationAmount = precipitationamount != null ? Optional.of(precipitationamount) : Optional.empty();
    this.weatherPhenomena = weatherPhenomena != null ? Optional.of(weatherPhenomena) : Optional.empty();
    this.opticalPhenomena = opticalPhenomena != null ? Optional.of(opticalPhenomena) : Optional.empty();
    this.artificialPhenomena = artificialPhenomena != null ? Optional.of(artificialPhenomena) : Optional.empty();
  }

  public Optional<AlternativeInput> getTemperature() {
    return temperature;
  }

  public Optional<SimLangEnums.CloudingTypes> getClouding() {
    return clouding;
  }

  public Optional<Sight> getSight() {
    return sight;
  }

  public Optional<SimLangEnums.PrecipitationTypes> getPrecipitationType() {
    return precipitationType;
  }

  public Optional<AlternativeInput> getHumidity() {
    return humidity;
  }

  public Optional<AlternativeInput> getPressure() {
    return pressure;
  }

  public Optional<AlternativeInput> getWindstrength() {
    return windStrength;
  }

  public Optional<AlternativeInput> getWinddirection() {
    return windDirection;
  }

  public Optional<AlternativeInput> getPrecipitationamount() {
    return precipitationAmount;
  }

  public Optional<ArrayList<WeatherPhenomenaInstance>> getWeatherPhenomena() {
    return weatherPhenomena;
  }

  public Optional<ArrayList<SimLangEnums.OpticalPhenomenas>> getOpticalPhenomena() {
    return opticalPhenomena;
  }

  public Optional<ArrayList<SimLangEnums.ArtificialPhenomena>> getArtificialPhenomena() {
    return artificialPhenomena;
  }
}
