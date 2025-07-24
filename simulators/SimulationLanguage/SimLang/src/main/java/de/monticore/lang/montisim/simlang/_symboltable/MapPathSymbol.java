/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;

public class MapPathSymbol extends CommonSymbol{

  public static final MapPathKind KIND = MapPathKind.INSTANCE;
  private String mapPath;

  public MapPathSymbol(String name, String mapPath) {
    super(name, KIND);
    this.mapPath = mapPath;
  }

  public String getMapPath() {
    return mapPath;
  }
}
