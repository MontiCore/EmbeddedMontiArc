/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class WindDirectionSymbol extends CommonSymbol{

  public static final WindDirectionKind KIND = WindDirectionKind.INSTANCE;
  private AlternativeInput windDirection;

  public WindDirectionSymbol(String name, AlternativeInput windDirection) {
    super(name, KIND);
    this.windDirection = windDirection;
  }

  public AlternativeInput getWindDirection() {
    return windDirection;
  }
}
