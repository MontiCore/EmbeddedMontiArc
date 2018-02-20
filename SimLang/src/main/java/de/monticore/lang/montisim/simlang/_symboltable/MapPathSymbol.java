package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.Time;

public class MapPathSymbol extends CommonSymbol{

  public static final MapPathSymbolKind KIND = MapPathSymbolKind.INSTANCE;
  private String mapPath;

  public MapPathSymbol(String name, String mapPath) {
    super(name, KIND);
    this.mapPath = mapPath;
  }

  public String getMapPath() {
    return mapPath;
  }
}
