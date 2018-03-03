package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.simlang.util.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class MapOverlapSymbol extends CommonSymbol{

  public static final MapOverlapSymbolKind KIND = MapOverlapSymbolKind.INSTANCE;
  private AlternativeInput mapOverlap;

  public MapOverlapSymbol(String name, AlternativeInput mapOverlap) {
    super(name, KIND);
    this.mapOverlap = mapOverlap;
  }

  public AlternativeInput getMapOverlap() {
    return mapOverlap;
  }
}
