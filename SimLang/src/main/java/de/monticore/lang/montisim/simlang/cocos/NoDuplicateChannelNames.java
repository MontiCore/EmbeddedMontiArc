/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulation;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationCoCo;
import de.monticore.lang.montisim.simlang._symboltable.ChannelSymbol;
import de.se_rwth.commons.logging.Log;
import java.util.ArrayList;
import java.util.Collection;

public class NoDuplicateChannelNames implements SimLangASTSimulationCoCo {

    @Override
    public void check(ASTSimulation node) {
      Collection<ChannelSymbol> channels = node.getEnclosingScope().get().resolveMany("channel", ChannelSymbol.KIND);
      ArrayList<String> names = new ArrayList<>();

      for(ChannelSymbol c : channels) {
        if(names.contains(c.getChannel().getName())) {
          Log.warn("Error: Duplicate Channel names.");
        }
        names.add(c.getChannel().getName());
      }
    }

}
