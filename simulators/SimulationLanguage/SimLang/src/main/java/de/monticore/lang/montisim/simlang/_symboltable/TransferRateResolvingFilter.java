/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class TransferRateResolvingFilter extends CommonResolvingFilter<TransferRateSymbol> {
  public TransferRateResolvingFilter() {
    super(TransferRateSymbol.KIND);
  }
}
