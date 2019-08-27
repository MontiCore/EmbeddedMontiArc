/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class MapOverlapResolvingFilter extends CommonResolvingFilter<MapOverlapSymbol> {
  public MapOverlapResolvingFilter() {
    super(MapOverlapSymbol.KIND);
  }
}
