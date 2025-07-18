/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for Time
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTMapName;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTMapNameCoCo;
import de.monticore.lang.montisim.simlang._symboltable.MapNameSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Arrays;

public class MapNameChecker implements SimLangASTMapNameCoCo {
  
  @Override
  public void check(ASTMapName node) {
    String[] allowedFormats = {"osm"};
    MapNameSymbol sym = (MapNameSymbol)node.getSymbol().get();
    
    if(!Arrays.asList(allowedFormats).contains(sym.getFileExtension())) {
      Log.warn("Fileformat Error: Invalid Mapfile format.");
    }
  }
}
