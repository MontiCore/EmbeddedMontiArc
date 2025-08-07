/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.Area;
import de.monticore.symboltable.CommonSymbol;

public class AreaSymbol extends CommonSymbol{

  public static final AreaKind KIND = AreaKind.INSTANCE;
  private Area area;

  public AreaSymbol(String name, Area area) {
    super(name, KIND);
    this.area = area;
  }

  public Area getArea() {
    return area;
  }
}
