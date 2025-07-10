/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import java.util.ArrayList;
import java.util.Collection;
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

  public ConcreteWeather(AlternativeInput temperature,
                         AlternativeInput humidity,
                         AlternativeInput pressure,
                         AlternativeInput windstrength,
                         AlternativeInput winddirection,
                         SimLangEnums.PrecipitationTypes precipitationtype,
                         AlternativeInput precipitationamount,
                         SimLangEnums.CloudingTypes clouding,
                         Sight sight,
                         ArrayList<WeatherPhenomenaInstance> weatherPhenomena,
                         ArrayList<SimLangEnums.OpticalPhenomenas> opticalPhenomena,
                         ArrayList<SimLangEnums.ArtificialPhenomena> artificialPhenomena) {

    this.temperature = temperature != null ? Optional.of(temperature) : Optional.empty();
    this.clouding = clouding != null ? Optional.of(clouding) : Optional.empty();
    this.sight = sight != null ? Optional.of(sight) : Optional.empty();
    this.precipitationType = precipitationtype != null ? Optional.of(precipitationtype) : Optional.empty();
    this.humidity = humidity != null ? Optional.of(humidity) : Optional.empty();
    this.pressure = pressure != null ? Optional.of(pressure) : Optional.empty();
    this.windStrength = windstrength != null ? Optional.of(windstrength) : Optional.empty();
    this.windDirection = winddirection != null ? Optional.of(winddirection) : Optional.empty();
    this.precipitationAmount = precipitationamount != null ? Optional.of(precipitationamount) : Optional.empty();

    if(weatherPhenomena != null && !weatherPhenomena.isEmpty()) {
      this.weatherPhenomena = Optional.of(weatherPhenomena);
    } else {
      this.weatherPhenomena = Optional.empty();
    }
    if(opticalPhenomena != null && !opticalPhenomena.isEmpty()) {
      this.opticalPhenomena = Optional.of(opticalPhenomena);
    } else {
      this.opticalPhenomena = Optional.empty();
    }
    if(artificialPhenomena != null && !artificialPhenomena.isEmpty()) {
      this.artificialPhenomena = Optional.of(artificialPhenomena);
    } else {
      this.artificialPhenomena = Optional.empty();
    }
  }

  public ConcreteWeather() {
    this.temperature = Optional.empty();
    this.humidity = Optional.empty();
    this.pressure = Optional.empty();
    this.windStrength = Optional.empty();
    this.windDirection = Optional.empty();
    this.precipitationType = Optional.empty();
    this.precipitationAmount = Optional.empty();
    this.clouding = Optional.empty();
    this.sight = Optional.empty();
    this.weatherPhenomena = Optional.empty();
    this.opticalPhenomena = Optional.empty();
    this.artificialPhenomena = Optional.empty();
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
