package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.simlang.util.Weather;

import java.util.ArrayList;

public class WeatherSymbol extends de.monticore.symboltable.CommonSymbol {

  public static final WeatherKind KIND = WeatherKind.INSTANCE;
  private final ArrayList<Weather> weathers;

  public WeatherSymbol(String name, ArrayList<Weather> weathers) {
    super(name, KIND);
    this.weathers = weathers;
  }
}
