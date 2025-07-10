/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.SimLangEnums;
import de.monticore.symboltable.CommonSymbol;

public class CloudingSymbol extends CommonSymbol{

  public static final CloudingKind KIND = CloudingKind.INSTANCE;
  private SimLangEnums.CloudingTypes clouding;

  public CloudingSymbol(String name, SimLangEnums.CloudingTypes clouding) {
    super(name, KIND);
    this.clouding = clouding;
  }

  public SimLangEnums.CloudingTypes getClouding() {
    return clouding;
  }
}
