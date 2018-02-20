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
import de.monticore.lang.montisim.simlang._ast.ASTSingleChannel;

public class NoDuplicateChannelNames implements SimLangASTSimulationCoCo {

    @Override
    public void check(ASTSimulation obj) {
        System.out.println("[CoCo] NoDuplicateChannelNames...");

        List<ASTChannel> channels = obj.getChannels();
        ArrayList<String> singleChannelNames = new ArrayList();
        ArrayList<ArrayList<String>> alternatingChannelNames = new ArrayList<>();
        ArrayList<String> newAlternatives;

        for(ASTChannel c : channels) {
          if(c.getSingleChannel().isPresent()) {
            if(!singleChannelNames.contains(c.getSingleChannel().get().getName())) {
              singleChannelNames.add(c.getSingleChannel().get().getName());
            } else {
              Log.error("Semantic Error: A channelname was defined more than once.");
            }
          }
          else if(c.getChannelList().isPresent()) {
            newAlternatives = new ArrayList<>();
            for(ASTSingleChannel ch : c.getChannelList().get().getSingleChannels()) {
              if(!singleChannelNames.contains(ch.getName())) {
                newAlternatives.add(ch.getName());
              } else {
                Log.error("Semantic Error: A channelname was defined more than once.");
              }
              for(ArrayList<String> altLists : alternatingChannelNames) {
                for(String name : altLists) {
                  if(ch.getName().equals(name)) {
                    Log.error("Semantic Error: A channelname was defined more than once.");
                  }
                }
                newAlternatives.add(ch.getName());
              }
            }
            alternatingChannelNames.add(newAlternatives);
          }
        }

        System.out.println("[Done] NoDuplicateChannelNames");
    }

}
