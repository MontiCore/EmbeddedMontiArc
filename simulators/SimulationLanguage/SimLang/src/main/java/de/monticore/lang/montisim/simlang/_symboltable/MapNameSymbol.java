/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;

public class MapNameSymbol extends CommonSymbol{

  public static final MapNameKind KIND = MapNameKind.INSTANCE;
  private String mapName;
  private String fileExtension;

  public MapNameSymbol(String name, String mapName, String ext) {
    super(name, KIND);
    this.mapName = mapName;
    this.fileExtension = ext;
  }

  public String getMapName() {
    return mapName;
  }
  public String getFileExtension() { return fileExtension; }
}
