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
    List<de.monticore.lang.montisim.weather._ast.ASTTemperature> temperatures = wObj.getTemperatures();
    List<de.monticore.lang.montisim.weather._ast.ASTClouding> cloudings = wObj.getCloudings();
    List<de.monticore.lang.montisim.weather._ast.ASTSight> sights = wObj.getSights();
    List<de.monticore.lang.montisim.weather._ast.ASTPrecipitationType> precipitationtypes = wObj.getPrecipitationTypes();
    List<de.monticore.lang.montisim.weather._ast.ASTHumidity> humiditys = wObj.getHumiditys();
    List<de.monticore.lang.montisim.weather._ast.ASTPressure> pressures = wObj.getPressures();
    List<de.monticore.lang.montisim.weather._ast.ASTWindStrength> windstrengths = wObj.getWindStrengths();
    List<de.monticore.lang.montisim.weather._ast.ASTWindDirection> winddirections = wObj.getWindDirections();
    List<de.monticore.lang.montisim.weather._ast.ASTPrecipitationAmount> precipitationamounts = wObj.getPrecipitationAmounts();
    
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
  }
  
}
