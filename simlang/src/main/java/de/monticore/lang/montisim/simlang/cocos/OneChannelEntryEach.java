/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTChannel;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTChannelCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.List;

public class OneChannelEntryEach implements SimLangASTChannelCoCo {
  
  @Override
  public void check(ASTChannel obj) {
    System.out.println("[CoCo] OneChannelEntryEach...");
    List<simlang._ast.ASTTransferrate> transferrates = obj.getTransferrates();
    List<simlang._ast.ASTLatency> latencys = obj.getLatencys();
    List<simlang._ast.ASTOutage> outages = obj.getOutages();
    List<simlang._ast.ASTArea> areas = obj.getAreas();
    
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
