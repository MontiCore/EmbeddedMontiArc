/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class WindStrengthSymbol extends CommonSymbol{

  public static final PressureKind KIND = PressureKind.INSTANCE;
  private AlternativeInput windStrength;

  public WindStrengthSymbol(String name, AlternativeInput windStrength) {
    super(name, KIND);
    this.windStrength = windStrength;
  }

  public AlternativeInput getWindStrength() {
    return windStrength;
  }
}
