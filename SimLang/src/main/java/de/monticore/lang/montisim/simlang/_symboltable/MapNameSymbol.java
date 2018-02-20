package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;

public class MapNameSymbol extends CommonSymbol{

  public static final MapNameSymbolKind KIND = MapNameSymbolKind.INSTANCE;
  private String mapName;

  public MapNameSymbol(String name, String mapName) {
    super(name, KIND);
    this.mapName = mapName;
  }

  public String getMapName() {
    return mapName;
  }
}
