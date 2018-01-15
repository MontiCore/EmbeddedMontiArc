/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package simlang.cocos;

import simlang._ast.ASTWeatherObj;
import simlang._cocos.SimLangASTWeatherObjCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.List;

public class NoMultipleWeatherObjEntries implements SimLangASTWeatherObjCoCo {
  
  @Override
  public void check(ASTWeatherObj wObj) {
    System.out.println("[CoCo] NoMultipleWeatherObjEntries...");
    
    List<weather._ast.ASTTemperature> temperatures = wObj.getTemperatures();
    List<weather._ast.ASTClouding> cloudings = wObj.getCloudings();
    List<weather._ast.ASTSight> sights = wObj.getSights();
    List<weather._ast.ASTPrecipitationtype> precipitationtypes = wObj.getPrecipitationtypes();
    List<weather._ast.ASTHumidity> humiditys = wObj.getHumiditys();
    List<weather._ast.ASTPressure> pressures = wObj.getPressures();
    List<weather._ast.ASTWindstrength> windstrengths = wObj.getWindstrengths();
    List<weather._ast.ASTWinddirection> winddirections = wObj.getWinddirections();
    List<weather._ast.ASTPrecipitationamount> precipitationamounts = wObj.getPrecipitationamounts();
    
    if(temperatures.size() > 1 |
       cloudings.size() > 1 |
       sights.size() > 1 |
       precipitationtypes.size() > 1 |
       humiditys.size() > 1 |
       pressures.size() > 1 |
       windstrengths.size() > 1 |
       winddirections.size() > 1 |
       precipitationamounts.size() > 1
    ) {
      Log.error("Semantic Error: A weather attribute was defined more than once.");
    }
    
    System.out.println("[Done] NoMultipleWeatherObjEntries");
  }
  
}
