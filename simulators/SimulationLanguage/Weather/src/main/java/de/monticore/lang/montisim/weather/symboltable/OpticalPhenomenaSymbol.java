/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.SimLangEnums;
import de.monticore.symboltable.CommonSymbol;

public class OpticalPhenomenaSymbol extends CommonSymbol{

  public static final OpticalPhenomenaKind KIND = OpticalPhenomenaKind.INSTANCE;
  private SimLangEnums.OpticalPhenomenas opticalPhenomena;

  public OpticalPhenomenaSymbol(String name, SimLangEnums.OpticalPhenomenas opticalPhenomena) {
    super(name, KIND);
    this.opticalPhenomena = opticalPhenomena;
  }

  public SimLangEnums.OpticalPhenomenas getOpticalPhenomena() {
    return opticalPhenomena;
  }
}
