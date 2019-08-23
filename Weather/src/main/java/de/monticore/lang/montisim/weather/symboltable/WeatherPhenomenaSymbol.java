/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.WeatherPhenomenaInstance;
import de.monticore.symboltable.CommonSymbol;

public class WeatherPhenomenaSymbol extends CommonSymbol{

  public static final WeatherPhenomenaKind KIND = WeatherPhenomenaKind.INSTANCE;
  private WeatherPhenomenaInstance weatherPhenomena;

  public WeatherPhenomenaSymbol(String name, WeatherPhenomenaInstance weatherPhenomena) {
    super(name, KIND);
    this.weatherPhenomena = weatherPhenomena;
  }

  public WeatherPhenomenaInstance getWeatherPhenomena() {
    return weatherPhenomena;
  }
}
