/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulation;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationCoCo;
import de.monticore.lang.montisim.simlang._symboltable.MapNameSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;

public class RequiredSimulationEntries implements SimLangASTSimulationCoCo {
  
  @Override
  public void check(ASTSimulation node) {
    Collection<MapNameSymbol> names = node.getSpannedScope().get().resolveMany("map_name",MapNameSymbol.KIND);
    
    if(names.size() < 1) {
      Log.warn("Semantic Error: map_name attribute missing.");
    }
  }
  
}
