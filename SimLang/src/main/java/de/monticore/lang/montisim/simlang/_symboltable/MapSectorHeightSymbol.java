package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.simlang.util.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class MapSectorHeightSymbol extends CommonSymbol{

  public static final MapSectorHeightSymbolKind KIND = MapSectorHeightSymbolKind.INSTANCE;
  private AlternativeInput mapSectorHeight;

  public MapSectorHeightSymbol(String name, AlternativeInput mapSectorHeight) {
    super(name, KIND);
    this.mapSectorHeight = mapSectorHeight;
  }

  public AlternativeInput getMapSectorHeight() {
    return mapSectorHeight;
  }
}
