package de.monticore.lang.montisim.simlang;

import de.monticore.lang.montisim.simlang._ast.ASTSimLangCompilationUnit;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.util.NoSuchElementException;

import static org.junit.Assert.assertTrue;


public class ASTTest {
  public static final boolean ENABLE_FAIL_QUICK = false;

  @Before
  public void setUp() {
    // ensure an empty log
    Log.getFindings().clear();
    Log.enableFailQuick(ENABLE_FAIL_QUICK);
  }

  @Test
  public void testAST() throws Exception {
    test("src/test/resources/test/ast/FullExample.sim");
    if (Log.getErrorCount() > 0) {
      throw new Exception("Test Failed, found errors");
    }
    Log.info("Completed: AST Test.","");
  }

  private void test(String file) {
    final ASTSimLangCompilationUnit ast = SimLangTool.parse(file);
    try {
      assertTrue(
              ast.getSimulation().getName().equals("FullExample") );
      assertTrue(
              ast.getSimulation().getSimulationRenderFrequencys().get(0).getTUnitNumber().get().equals("60ms") );
      assertTrue(
              ast.getSimulation().getSimulationLoopFrequencys().get(0).getRange().get().getStart().get().getUnitNumber().get().getTUnitNumber().get().equals("60m") );
      assertTrue(
              ast.getSimulation().getSimulationLoopFrequencys().get(0).getRange().get().getStep().get().getUnitNumber().get().getTUnitNumber().get().equals("1m") );
      assertTrue(
              ast.getSimulation().getSimulationLoopFrequencys().get(0).getRange().get().getEnd().get().getUnitNumber().get().getTUnitNumber().get().equals("80m") );
      assertTrue(
              ast.getSimulation().getSimulationDurations().get(0).getTUnitNumberList().get().getTUnitNumbers().get(0).equals("50m") );
      assertTrue(
              ast.getSimulation().getSimulationDurations().get(0).getTUnitNumberList().get().getTUnitNumbers().get(1).equals("4h") );
      assertTrue(
              ast.getSimulation().getSimulationDurations().get(0).getTUnitNumberList().get().getTUnitNumbers().get(2).equals("5h") );
      assertTrue(
              ast.getSimulation().getSimulationDurations().get(0).getTUnitNumberList().get().getTUnitNumbers().get(3).equals("6h") );
      //assertTrue(ast.getSimulation().getSimulationTypes().get(0).getSimType() == 2 ); //MC has unintuitive enums
      //assertTrue(ast.getSimulation().getWeathers().get(0).getSingleWeather().get().getFixedWeather().get().getFixWeatherObj().getTemperatures().get(0).getWeatherTemperature().equals("290°") ); //needs antlr 4.7.1
      assertTrue(ast.getSimulation().getWeathers().get(0).getSingleWeather().get().getFixedWeather().get().getFixWeatherObj().getHumiditys().get(0).getWeatherHumidity().equals("0.2") );
      assertTrue(
              ast.getSimulation().getWeathers().get(0).getSingleWeather().get().getFixedWeather().get().getFixWeatherObj().getPressures().get(0).getWeatherPressure().equals("100Pa") );
      assertTrue(
              ast.getSimulation().getWeathers().get(0).getSingleWeather().get().getFixedWeather().get().getFixWeatherObj().getWindstrengths().get(0).getWeatherWindstrength().equals("2km/h") );
      assertTrue(
              ast.getSimulation().getWeathers().get(0).getSingleWeather().get().getFixedWeather().get().getFixWeatherObj().getWinddirections().get(0).getWeatherWinddirection().equals("38.0°") );
      //assertTrue(ast.getSimulation().getWeathers().get(0).getSingleWeather().get().getFixedWeather().get().getFixWeatherObj().getPrecipitationtypes().get(0).getPrecipitationType() == 5 );
      assertTrue(
              ast.getSimulation().getWeathers().get(0).getSingleWeather().get().getFixedWeather().get().getFixWeatherObj().getPrecipitationamounts().get(0).getWeatherPrecipitationamount().equals("5mm") );
      //assertTrue(ast.getSimulation().getWeathers().get(0).getSingleWeather().get().getFixedWeather().get().getFixWeatherObj().getCloudings().get(0).getCloudingType() == 5 );
      assertTrue(
              ast.getSimulation().getWeathers().get(0).getSingleWeather().get().getFixedWeather().get().getFixWeatherObj().getSights().get(0).isUnlimited() );
      //add phenomena
      assertTrue(
              ast.getSimulation().getTimes().get(0).getSingleTime().get().getHours().equals("1") );
      assertTrue(
              ast.getSimulation().getTimes().get(0).getSingleTime().get().getMinutes().equals("22") );
      assertTrue(
              ast.getSimulation().getTimes().get(0).getSingleTime().get().getSeconds().get().equals("33") );
      assertTrue(
              ast.getSimulation().getTimes().get(0).getSingleTime().get().getMilliseconds().get().equals("444") );
      assertTrue(
              ast.getSimulation().getMapPaths().get(0).getMapPath().equals("Maps") );
      assertTrue(
              ast.getSimulation().getMapNames().get(0).getMapName().equals("HorsterDreieck") );
      assertTrue(
              ast.getSimulation().getMapNames().get(0).getFileFormat().equals("osm") );
      //assertTrue(ast.getSimulation().getMapHeights().get(0).getHeightMode() == 0 );
      assertTrue(
              ast.getSimulation().getMapOverlaps().get(0).getTUnitNumber().get().equals("10") );
      assertTrue(
              ast.getSimulation().getMapSectorWidths().get(0).getTUnitNumber().get().equals("100") );
      assertTrue(
              ast.getSimulation().getMapSectorHeights().get(0).getTUnitNumber().get().equals("100") );
      assertTrue(
              ast.getSimulation().getMaxSectorUserss().get(0).getTUnitNumber().get().equals("1234") );
      assertTrue(
              ast.getSimulation().getTimeouts().get(0).getTUnitNumber().get().equals("12h") );
      assertTrue(
              ast.getSimulation().getGravitys().get(0).getTUnitNumber().get().equals("12m/s^2") );
      //assertTrue(ast.getSimulation().getPedestrianDensitys().get(0).getLambda().get().getHours().equals("1") );
      assertTrue(
              ast.getSimulation().getPedestrianss().get(0).getStartX().equals("10.10") );
      assertTrue(
              ast.getSimulation().getPedestrianss().get(0).getStartY().equals("10.0") );
      assertTrue(
              ast.getSimulation().getPedestrianss().get(0).getStartZ().get().equals("10") );
      assertTrue(
              ast.getSimulation().getPedestrianss().get(0).getDestX().equals("0") );
      assertTrue(
              ast.getSimulation().getPedestrianss().get(0).getDestY().equals("20.0") );
      assertTrue(ast.getSimulation().getPedestrianss().get(0).getDestZ().get().equals("0.0") );
      assertTrue(
              ast.getSimulation().getVehicless().get(0).getExplicitVehicle().get().getVehicle().equals("car1") );
      assertTrue(
              ast.getSimulation().getVehicless().get(0).getExplicitVehicle().get().getStartX().equals("-22") );
      assertTrue(
              ast.getSimulation().getVehicless().get(0).getExplicitVehicle().get().getStartY().equals("-34.0") );
      assertTrue(
              ast.getSimulation().getVehicless().get(0).getExplicitVehicle().get().getStartRot().equals("90") );
      assertTrue(
              ast.getSimulation().getVehicless().get(0).getExplicitVehicle().get().getDestX().equals("-1") );
      assertTrue(
              ast.getSimulation().getVehicless().get(0).getExplicitVehicle().get().getDestY().equals("0") );
      assertTrue(ast.getSimulation().getVehicless().get(0).getExplicitVehicle().get().getDestZ().get().equals("10") );
      assertTrue(
              ast.getSimulation().getVehicless().get(1).getPathedVehicle().get().getStartX().equals("123") );
      assertTrue(
              ast.getSimulation().getVehicless().get(1).getPathedVehicle().get().getStartY().equals("-94") );
      assertTrue(
              ast.getSimulation().getVehicless().get(1).getPathedVehicle().get().getSpawnRadius().equals("201") );
      assertTrue(
              ast.getSimulation().getVehicless().get(1).getPathedVehicle().get().getDestX().equals("1024") );
      assertTrue(
              ast.getSimulation().getVehicless().get(1).getPathedVehicle().get().getDestY().equals("960") );
      assertTrue(
              ast.getSimulation().getVehicless().get(1).getPathedVehicle().get().getDestRadius().equals("200") );
      assertTrue(
              ast.getSimulation().getVehicless().get(2).getRandomVehicle().get().getAmount().equals("1000") );
      assertTrue(
              ast.getSimulation().getVehicless().get(3).getRandomVehicle().get().getAmount().equals("1234") );
      assertTrue(
              ast.getSimulation().getVehicless().get(3).getRandomVehicle().get().getStartX().get().equals("-150") );
      assertTrue(
              ast.getSimulation().getVehicless().get(3).getRandomVehicle().get().getStartY().get().equals("-150") );
      assertTrue(
              ast.getSimulation().getVehicless().get(3).getRandomVehicle().get().getDestX().get().equals("500") );
      assertTrue(
              ast.getSimulation().getVehicless().get(3).getRandomVehicle().get().getDestY().get().equals("600") );
      assertTrue(
              ast.getSimulation().getChannels().get(0).getSingleChannel().get().getName().equals("LTE") );
      assertTrue(
              ast.getSimulation().getChannels().get(0).getSingleChannel().get().isFixed() );
      assertTrue(
              ast.getSimulation().getChannels().get(0).getSingleChannel().get().getTransferrates().get(0).getTUnitNumber().get().equals("20 Mbit/s") );
      assertTrue(
              ast.getSimulation().getChannels().get(0).getSingleChannel().get().getLatencys().get(0).getTUnitNumber().get().equals("10ms") );
      assertTrue(
              ast.getSimulation().getChannels().get(0).getSingleChannel().get().getOutages().get(0).getTUnitNumber().get().equals("0.001") );
      assertTrue(
              ast.getSimulation().getChannels().get(0).getSingleChannel().get().getAreas().get(0).isGlobal());
    }
    catch (NoSuchElementException e) {
      Log.error("Error in ASTTest: Tried accessing an Optional's content without it being present.", e);
    }
    catch (IndexOutOfBoundsException e) {
      Log.error("Error in ASTTest: Tried accessing a list index that does not exist.", e);
    }
    catch (Exception e) {
      Log.error("Error in ASTTest: Unconsidered error or parse error (check your input).", e);
    }
  }
}
