/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.weather._ast.ASTWeatherScope;
import de.monticore.lang.montisim.weather._cocos.WeatherASTWeatherScopeCoCo;
import de.monticore.lang.montisim.weather.symboltable.TemperatureSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;

public class NoMultipleWeatherObjEntries implements WeatherASTWeatherScopeCoCo {
  
  @Override
  public void check(ASTWeatherScope node) {
    Collection<TemperatureSymbol> temp = node.getSpannedScope().get().resolveMany("temperature", TemperatureSymbol.KIND);
    Collection<TemperatureSymbol> cloud = node.getSpannedScope().get().resolveMany("clouding", TemperatureSymbol.KIND);
    Collection<TemperatureSymbol> humidity = node.getSpannedScope().get().resolveMany("humidity", TemperatureSymbol.KIND);
    Collection<TemperatureSymbol> pressure = node.getSpannedScope().get().resolveMany("pressure", TemperatureSymbol.KIND);
    Collection<TemperatureSymbol> windS = node.getSpannedScope().get().resolveMany("wind_strength", TemperatureSymbol.KIND);
    Collection<TemperatureSymbol> windD = node.getSpannedScope().get().resolveMany("wind_direction", TemperatureSymbol.KIND);
    Collection<TemperatureSymbol> preT = node.getSpannedScope().get().resolveMany("precipitation_type", TemperatureSymbol.KIND);
    Collection<TemperatureSymbol> preA = node.getSpannedScope().get().resolveMany("precipitation_amount", TemperatureSymbol.KIND);
    Collection<TemperatureSymbol> sight = node.getSpannedScope().get().resolveMany("sight", TemperatureSymbol.KIND);
    
    if(temp.size() > 1 |
       cloud.size() > 1 |
       humidity.size() > 1 |
       pressure.size() > 1 |
       windS.size() > 1 |
       windD.size() > 1 |
       preT.size() > 1 |
       preA.size() > 1 |
       sight.size() > 1
    ) {
      Log.warn("Semantic Error: A weather attribute was defined more than once.");
    }
  }
  
}
