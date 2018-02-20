/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTWeatherObj;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTWeatherObjCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.List;

public class NoMultipleWeatherObjEntries implements SimLangASTWeatherObjCoCo {
  
  @Override
  public void check(ASTWeatherObj wObj) {
    System.out.println("[CoCo] NoMultipleWeatherObjEntries...");
    
    List<de.monticore.lang.montisim.weather._ast.ASTTemperature> temperatures = wObj.getTemperatures();
    List<de.monticore.lang.montisim.weather._ast.ASTClouding> cloudings = wObj.getCloudings();
    List<de.monticore.lang.montisim.weather._ast.ASTSight> sights = wObj.getSights();
    List<de.monticore.lang.montisim.weather._ast.ASTPrecipitationtype> precipitationtypes = wObj.getPrecipitationtypes();
    List<de.monticore.lang.montisim.weather._ast.ASTHumidity> humiditys = wObj.getHumiditys();
    List<de.monticore.lang.montisim.weather._ast.ASTPressure> pressures = wObj.getPressures();
    List<de.monticore.lang.montisim.weather._ast.ASTWindstrength> windstrengths = wObj.getWindstrengths();
    List<de.monticore.lang.montisim.weather._ast.ASTWinddirection> winddirections = wObj.getWinddirections();
    List<de.monticore.lang.montisim.weather._ast.ASTPrecipitationamount> precipitationamounts = wObj.getPrecipitationamounts();
    
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
