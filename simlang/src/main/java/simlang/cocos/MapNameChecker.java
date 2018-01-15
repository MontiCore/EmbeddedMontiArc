/*
 * Custom CoCos for Time
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package simlang.cocos;

import simlang._ast.ASTMapName;
import simlang._cocos.SimLangASTMapNameCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.Arrays;

public class MapNameChecker implements SimLangASTMapNameCoCo {
  
  @Override
  public void check(ASTMapName obj) {
    System.out.println("[CoCo] MapNameChecker...");
    
    String[] allowedFormats = {"osm"};
    
    
    if(!Arrays.asList(allowedFormats).contains(obj.getFileFormat())) {
      Log.error("Fileformat Error: Invalid Mapfile format.");
    }
    
    
    System.out.println("[Done] MapNameChecker");
  }
  
}
