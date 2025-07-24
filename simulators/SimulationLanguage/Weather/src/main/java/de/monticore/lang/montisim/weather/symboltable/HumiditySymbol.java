/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class HumiditySymbol extends CommonSymbol{

  public static final HumidityKind KIND = HumidityKind.INSTANCE;
  private AlternativeInput humidity;

  public HumiditySymbol(String name, AlternativeInput humidity) {
    super(name, KIND);
    this.humidity = humidity;
  }

  public AlternativeInput getHumidity() {
    return humidity;
  }
}
