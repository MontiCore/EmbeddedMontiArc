/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.Weather;

import java.util.ArrayList;

public class WeatherSymbol extends de.monticore.symboltable.CommonSymbol {

  public static final WeatherKind KIND = WeatherKind.INSTANCE;
  private ArrayList<Weather> weathers;

  public WeatherSymbol(String name, ArrayList<Weather> weathers) {
    super(name, KIND);
    this.weathers = weathers;
  }
  public WeatherSymbol(String name) {
    super(name, KIND);
  }

  public ArrayList<Weather> getWeathers() {
    return this.weathers;
  }

  public void setWeathers(ArrayList<Weather> weathers) {
    this.weathers = weathers;
  }
}
