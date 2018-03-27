/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulation;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.List;
import java.util.ArrayList;
import de.monticore.lang.montisim.simlang._ast.ASTChannel;

public class NoDuplicateChannelNames implements SimLangASTSimulationCoCo {

    @Override
    public void check(ASTSimulation obj) {
        List<ASTChannel> channels = obj.getChannels();
        ArrayList<String> names = new ArrayList<>();

        for(ASTChannel c : channels) {
          if(names.contains(c.getName())) {
            Log.error("Error: Duplicate Channel names.");
          }
          names.add(c.getName());
        }
    }

}
