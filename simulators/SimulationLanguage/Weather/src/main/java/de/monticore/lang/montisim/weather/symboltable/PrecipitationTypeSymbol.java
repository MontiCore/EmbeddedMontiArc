/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.SimLangEnums;
import de.monticore.symboltable.CommonSymbol;

public class PrecipitationTypeSymbol extends CommonSymbol{

  public static final PrecipitationTypeKind KIND = PrecipitationTypeKind.INSTANCE;
  private SimLangEnums.PrecipitationTypes precipitationType;

  public PrecipitationTypeSymbol(String name, SimLangEnums.PrecipitationTypes precipitationType) {
    super(name, KIND);
    this.precipitationType = precipitationType;
  }

  public SimLangEnums.PrecipitationTypes getPrecipitationType() {
    return precipitationType;
  }
}
