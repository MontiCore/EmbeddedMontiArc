/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class MapSectorWidthResolvingFilter extends CommonResolvingFilter<MapSectorWidthSymbol> {
  public MapSectorWidthResolvingFilter() {
    super(MapSectorWidthSymbol.KIND);
  }
}
