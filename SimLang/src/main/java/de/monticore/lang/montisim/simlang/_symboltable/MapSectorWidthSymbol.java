package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.ValueListRangeLambda;

public class MapSectorWidthSymbol extends CommonSymbol{

  public static final MapSectorWidthSymbolKind KIND = MapSectorWidthSymbolKind.INSTANCE;
  private ValueListRangeLambda mapSectorWidth;

  public MapSectorWidthSymbol(String name, ValueListRangeLambda mapSectorWidth) {
    super(name, KIND);
    this.mapSectorWidth = mapSectorWidth;
  }

  public ValueListRangeLambda getMapSectorWidth() {
    return mapSectorWidth;
  }
}
