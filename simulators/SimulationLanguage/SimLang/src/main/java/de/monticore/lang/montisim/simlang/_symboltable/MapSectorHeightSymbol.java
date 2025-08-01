/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class MapSectorHeightSymbol extends CommonSymbol{

  public static final MapSectorHeightKind KIND = MapSectorHeightKind.INSTANCE;
  private AlternativeInput mapSectorHeight;

  public MapSectorHeightSymbol(String name, AlternativeInput mapSectorHeight) {
    super(name, KIND);
    this.mapSectorHeight = mapSectorHeight;
  }

  public AlternativeInput getMapSectorHeight() {
    return mapSectorHeight;
  }
}
