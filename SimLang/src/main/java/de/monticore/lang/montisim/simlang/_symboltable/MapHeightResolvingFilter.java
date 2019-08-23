/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class MapHeightResolvingFilter extends CommonResolvingFilter<MapHeightSymbol> {
  public MapHeightResolvingFilter() {
    super(MapHeightSymbol.KIND);
  }
}
