/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTChannel;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTChannelCoCo;
import de.monticore.lang.montisim.simlang._symboltable.AreaSymbol;
import de.monticore.lang.montisim.simlang._symboltable.LatencySymbol;
import de.monticore.lang.montisim.simlang._symboltable.OutageSymbol;
import de.monticore.lang.montisim.simlang._symboltable.TransferRateSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;

public class OneChannelEntryEach implements SimLangASTChannelCoCo {
  
  @Override
  public void check(ASTChannel node) {
    Collection<TransferRateSymbol> tR = node.getSpannedScope().get().resolveMany("transfer_rate", TransferRateSymbol.KIND);
    Collection<LatencySymbol> la = node.getSpannedScope().get().resolveMany("latency", LatencySymbol.KIND);
    Collection<OutageSymbol> ou = node.getSpannedScope().get().resolveMany("outage", OutageSymbol.KIND);
    Collection<AreaSymbol> ar = node.getSpannedScope().get().resolveMany("area", AreaSymbol.KIND);

    if(tR.size() != 1 |
       la.size() != 1 |
       ou.size() != 1 |
       ar.size() != 1
    ) {
      Log.warn("Semantic Error: A channel attribute was defined more than once or not at all.");
    }
  }
  
}
