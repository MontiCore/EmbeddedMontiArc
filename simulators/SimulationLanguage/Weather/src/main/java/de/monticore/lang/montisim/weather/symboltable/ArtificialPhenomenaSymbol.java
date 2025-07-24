/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.lang.montisim.util.types.SimLangEnums;
import de.monticore.symboltable.CommonSymbol;

public class ArtificialPhenomenaSymbol extends CommonSymbol{

  public static final ArtificialPhenomenaKind KIND = ArtificialPhenomenaKind.INSTANCE;
  private SimLangEnums.ArtificialPhenomena artificialPhenomena;

  public ArtificialPhenomenaSymbol(String name, SimLangEnums.ArtificialPhenomena artificialPhenomena) {
    super(name, KIND);
    this.artificialPhenomena = artificialPhenomena;
  }

  public SimLangEnums.ArtificialPhenomena getArtificialPhenomena() {
    return artificialPhenomena;
  }
}
