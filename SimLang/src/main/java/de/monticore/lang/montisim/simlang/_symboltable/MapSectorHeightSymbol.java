package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.ValueListRangeLambda;

public class MapSectorHeightSymbol extends CommonSymbol{

  public static final MapSectorHeightSymbolKind KIND = MapSectorHeightSymbolKind.INSTANCE;
  private ValueListRangeLambda mapSectorHeight;

  public MapSectorHeightSymbol(String name, ValueListRangeLambda mapSectorHeight) {
    super(name, KIND);
    this.mapSectorHeight = mapSectorHeight;
  }

  public ValueListRangeLambda getMapSectorHeight() {
    return mapSectorHeight;
  }
}
