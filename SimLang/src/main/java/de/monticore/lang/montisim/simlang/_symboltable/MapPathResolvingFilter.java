/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class MapPathResolvingFilter extends CommonResolvingFilter<MapPathSymbol> {
  public MapPathResolvingFilter() {
    super(MapPathSymbol.KIND);
  }
}
