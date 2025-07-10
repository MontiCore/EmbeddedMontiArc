/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class TransferRateSymbol extends CommonSymbol{

  public static final TransferRateKind KIND = TransferRateKind.INSTANCE;
  private AlternativeInput transferRate;

  public TransferRateSymbol(String name, AlternativeInput transferRate) {
    super(name, KIND);
    this.transferRate = transferRate;
  }

  public AlternativeInput getTransferRate() {
    return transferRate;
  }
}
