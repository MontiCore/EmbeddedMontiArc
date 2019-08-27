/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class MapSectorHeightResolvingFilter extends CommonResolvingFilter<MapSectorHeightSymbol> {
  public MapSectorHeightResolvingFilter() {
    super(MapSectorHeightSymbol.KIND);
  }
}
