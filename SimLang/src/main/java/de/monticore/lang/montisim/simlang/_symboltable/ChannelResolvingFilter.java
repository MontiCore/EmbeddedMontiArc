/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class ChannelResolvingFilter extends CommonResolvingFilter<ChannelSymbol> {
  public ChannelResolvingFilter() {
    super(ChannelSymbol.KIND);
  }
}
