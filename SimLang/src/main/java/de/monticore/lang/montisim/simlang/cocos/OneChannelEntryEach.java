/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSingleChannel;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSingleChannelCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.List;

public class OneChannelEntryEach implements SimLangASTSingleChannelCoCo {
  
  @Override
  public void check(ASTSingleChannel obj) {
    System.out.println("[CoCo] OneChannelEntryEach...");
    List<de.monticore.lang.montisim.simlang._ast.ASTTransferrate> transferrates = obj.getTransferrates();
    List<de.monticore.lang.montisim.simlang._ast.ASTLatency> latencys = obj.getLatencys();
    List<de.monticore.lang.montisim.simlang._ast.ASTOutage> outages = obj.getOutages();
    List<de.monticore.lang.montisim.simlang._ast.ASTArea> areas = obj.getAreas();
    
    if(transferrates.size() != 1 |
       latencys.size() != 1 |
       outages.size() != 1 |
       areas.size() != 1
    ) {
      Log.error("Semantic Error: A channel attribute was defined more than once or not at all.");
    }
    
    System.out.println("[Done] OneChannelEntryEach");
  }
  
}
