/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class PressureSymbol extends CommonSymbol{

  public static final PressureKind KIND = PressureKind.INSTANCE;
  private AlternativeInput pressure;

  public PressureSymbol(String name, AlternativeInput pressure) {
    super(name, KIND);
    this.pressure = pressure;
  }

  public AlternativeInput getPressure() {
    return pressure;
  }
}
