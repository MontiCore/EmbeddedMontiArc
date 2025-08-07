/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.util.types.MapHeight;

public class MapHeightSymbol extends CommonSymbol{

  public static final MapHeightKind KIND = MapHeightKind.INSTANCE;
  private MapHeight mapHeight;

  public MapHeightSymbol(String name, MapHeight mapHeight) {
    super(name, KIND);
    this.mapHeight = mapHeight;
  }

  public MapHeight getMapHeight() {
    return mapHeight;
  }
}
