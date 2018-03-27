/*
 * Custom CoCos for Time
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTMapName;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTMapNameCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.Arrays;

public class MapNameChecker implements SimLangASTMapNameCoCo {
  
  @Override
  public void check(ASTMapName obj) {
    String[] allowedFormats = {"osm"};
    
    if(!Arrays.asList(allowedFormats).contains(obj.getFileFormat())) {
      Log.error("Fileformat Error: Invalid Mapfile format.");
    }
  }
}
