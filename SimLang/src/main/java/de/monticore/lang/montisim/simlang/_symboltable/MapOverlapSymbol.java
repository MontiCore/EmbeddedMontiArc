package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.ValueListRangeLambda;

public class MapOverlapSymbol extends CommonSymbol{

  public static final MapOverlapSymbolKind KIND = MapOverlapSymbolKind.INSTANCE;
  private ValueListRangeLambda mapOverlap;

  public MapOverlapSymbol(String name, ValueListRangeLambda mapOverlap) {
    super(name, KIND);
    this.mapOverlap = mapOverlap;
  }

  public ValueListRangeLambda getMapOverlap() {
    return mapOverlap;
  }
}
