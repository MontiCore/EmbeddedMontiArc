/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.Sight;
import de.monticore.symboltable.CommonSymbol;

public class SightSymbol extends CommonSymbol{

  public static final SightKind KIND = SightKind.INSTANCE;
  private Sight sight;

  public SightSymbol(String name, Sight sight) {
    super(name, KIND);
    this.sight = sight;
  }

  public Sight getSight() {
    return sight;
  }
}
