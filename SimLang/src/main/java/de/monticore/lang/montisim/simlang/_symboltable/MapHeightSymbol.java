package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.MapHeight;

public class MapHeightSymbol extends CommonSymbol{

  public static final MapHeightSymbolKind KIND = MapHeightSymbolKind.INSTANCE;
  private MapHeight mapHeight;

  public MapHeightSymbol(String name, MapHeight mapHeight) {
    super(name, KIND);
    this.mapHeight = mapHeight;
  }

  public MapHeight getMapHeight() {
    return mapHeight;
  }
}
