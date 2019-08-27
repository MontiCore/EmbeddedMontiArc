/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class MapSectorWidthSymbol extends CommonSymbol{

  public static final MapSectorWidthKind KIND = MapSectorWidthKind.INSTANCE;
  private AlternativeInput mapSectorWidth;

  public MapSectorWidthSymbol(String name, AlternativeInput mapSectorWidth) {
    super(name, KIND);
    this.mapSectorWidth = mapSectorWidth;
  }

  public AlternativeInput getMapSectorWidth() {
    return mapSectorWidth;
  }
}
