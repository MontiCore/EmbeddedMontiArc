/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class TemperatureSymbol extends CommonSymbol{

  public static final HumidityKind KIND = HumidityKind.INSTANCE;
  private AlternativeInput temperature;

  public TemperatureSymbol(String name, AlternativeInput temperature) {
    super(name, KIND);
    this.temperature = temperature;
  }

  public AlternativeInput getTemperature() {
    return temperature;
  }
}
