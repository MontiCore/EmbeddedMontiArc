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
import java.util.Arrays;

public class NoDuplicateChannelNames implements SimLangASTSimulationCoCo {

    @Override
    public void check(ASTSimulation obj) {
        System.out.println("[CoCo] NoMultipleSimlulationEntries...");

        List<?> channels = obj.getChannels();
        ArrayList<String> channelNames = new ArrayList();

        for(ASTChannel c : channels) {
            if(!Arrays.toList(channelNames).contains(c.getChannelName())) {
                channelNames.add(c.getChannelName());
            } else {
                Log.error("Semantic Error: A channelname was defined more than once.");
            }
        }

        if() {
            Log.error("Semantic Error: A simulation attribute was defined more than once.");
        }

        System.out.println("[Done] NoMultipleSimulationEntries");
    }

}
