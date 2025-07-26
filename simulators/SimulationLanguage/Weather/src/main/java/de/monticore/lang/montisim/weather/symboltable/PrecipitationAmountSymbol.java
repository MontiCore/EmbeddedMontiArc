/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class PrecipitationAmountSymbol extends CommonSymbol{

  public static final PrecipitationAmountKind KIND = PrecipitationAmountKind.INSTANCE;
  private AlternativeInput precipitationAmount;

  public PrecipitationAmountSymbol(String name, AlternativeInput precipitationAmount) {
    super(name, KIND);
    this.precipitationAmount = precipitationAmount;
  }

  public AlternativeInput getPrecipitationAmount() {
    return precipitationAmount;
  }
}
