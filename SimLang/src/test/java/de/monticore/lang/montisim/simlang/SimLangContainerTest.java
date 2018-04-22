package de.monticore.lang.montisim.simlang;

import de.monticore.lang.montisim.simlang.adapter.SimLangContainer;
import de.monticore.lang.montisim.util.types.SimLangEnums;

import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

public class SimLangContainerTest {
  @Before
  public void setUp() {
    // ensure an empty log
    Log.getFindings().clear();
    Log.enableFailQuick(false);
  }

  @Test
  public void testSimLangContainer() {
    final SimLangContainer adapter = SimLangTool.parseIntoContainer("src/test/resources/test/ast/ASTTest.sim");

    assert adapter.getSimulationRenderFrequency().isPresent();
    assert adapter.getSimulationRenderFrequency().get().getNUnit().get().getNumberUnit().equals("60.0ms");

    assert adapter.getSimulationLoopFrequency().isPresent();
    assert adapter.getSimulationLoopFrequency().get().getRange().get().getStart().getNumberUnit().equals("60.0m");
    assert adapter.getSimulationLoopFrequency().get().getRange().get().getStep().getNumberUnit().equals("1.0m");
    assert adapter.getSimulationLoopFrequency().get().getRange().get().getEnd().getNumberUnit().equals("80.0m");

    assert adapter.getSimulationDuration().isPresent();
    assert adapter.getSimulationDuration().get().getList().get().get(0).getNumberUnit().equals("50.0m");
    assert adapter.getSimulationDuration().get().getList().get().get(1).getNumberUnit().equals("4.0h");
    assert adapter.getSimulationDuration().get().getList().get().get(2).getNumberUnit().equals("5.0h");
    assert adapter.getSimulationDuration().get().getList().get().get(3).getNumberUnit().equals("6.0h");

    assert adapter.getSimulationType().isPresent();
    //assert adapter.getSimulationType().get().equals(SimLangEnums.SimulationTypes.MAXFPS);

    assert adapter.getWeather().isPresent();
    assert adapter.getWeather().get().get(0).getFixedWeather().get().getWeather().getTemperature().get().getNUnit().get().getNumberUnit().equals("290.0K");
    assert adapter.getWeather().get().get(0).getFixedWeather().get().getWeather().getHumidity().get().getNUnit().get().getNumberUnit().equals("0.2");
    assert adapter.getWeather().get().get(0).getFixedWeather().get().getWeather().getPressure().get().getNUnit().get().getNumberUnit().equals("100.0Pa");
    assert adapter.getWeather().get().get(0).getFixedWeather().get().getWeather().getWindstrength().get().getNUnit().get().getNumberUnit().equals("2.0km/h");
    //assert adapter.getWeather().get().get(0).getFixedWeather().get().getWeather().getPrecipitationType().get().equals(SimLangEnums.PrecipitationTypes.SNOW);
    assert adapter.getWeather().get().get(0).getFixedWeather().get().getWeather().getPrecipitationamount().get().getNUnit().get().getNumberUnit().equals("5.0mm");
    //assert adapter.getWeather().get().get(0).getFixedWeather().get().getWeather().getClouding().get().equals(SimLangEnums.CloudingTypes.STRATUS);
    assert adapter.getWeather().get().get(0).getFixedWeather().get().getWeather().getSight().get().isUnlimited();

    assert adapter.getTime().isPresent();
    assert adapter.getTime().get().get(0).getHours() == 1;
    assert adapter.getTime().get().get(0).getMinutes() == 22;
    assert adapter.getTime().get().get(0).getSeconds().get() == 33;
    assert adapter.getTime().get().get(0).getMilliseconds().get() == 444;

    assert adapter.getMapPath().isPresent();
    assert adapter.getMapPath().get().equals("Maps");

    assert adapter.getMapName().isPresent();
    assert adapter.getMapName().get().equals("HorsterDreieck");

    assert adapter.getMapHeight().isPresent();
    //assert adapter.getMapHeight().get().getHeightMode().get().equals(SimLangEnums.SimulationHeightModes.FLAT);

    assert adapter.getMapOverlap().isPresent();
    assert adapter.getMapOverlap().get().getNUnit().get().getNumberUnit().equals("10.0");

    assert adapter.getMapSectorWidth().isPresent();
    assert adapter.getMapSectorWidth().get().getNUnit().get().getNumberUnit().equals("100.0");

    assert adapter.getMapSectorHeight().isPresent();
    assert adapter.getMapSectorHeight().get().getNUnit().get().getNumberUnit().equals("100.0");

    assert adapter.getMaxSectorUsers().isPresent();
    assert adapter.getMaxSectorUsers().get().getNUnit().get().getNumberUnit().equals("1234.0");

    assert adapter.getTimeout().isPresent();
    assert adapter.getTimeout().get().getNUnit().get().getNumberUnit().equals("12.0h");

    assert adapter.getGravity().isPresent();
    assert adapter.getGravity().get().getNUnit().get().getNumberUnit().equals("12.0m/s²");

    assert adapter.getPedestrianDensity().isPresent();
    assert adapter.getPedestrianDensity().get().getNUnit().get().getNumberUnit().equals("2.0");

    assert adapter.getPedestrians().isPresent();
    assert adapter.getPedestrians().get().get(0).getStartX() == 10.10f;
    assert adapter.getPedestrians().get().get(0).getStartY() == 10.0f;
    assert adapter.getPedestrians().get().get(0).getStartZ().get() == 10.0f;
    assert adapter.getPedestrians().get().get(0).getDestX() == 0.0f;
    assert adapter.getPedestrians().get().get(0).getDestY() == 20.0f;
    assert adapter.getPedestrians().get().get(0).getDestZ().get() == 0.0f;

    assert adapter.getExplicitVehicles().isPresent();
    assert adapter.getExplicitVehicles().get().get(0).getName().equals("car1");
    assert adapter.getExplicitVehicles().get().get(0).getPath().getStartX() == -22.0f;
    assert adapter.getExplicitVehicles().get().get(0).getPath().getStartY() == -34.0f;
    assert adapter.getExplicitVehicles().get().get(0).getPath().getDestX() == -1.0f;
    assert adapter.getExplicitVehicles().get().get(0).getPath().getDestY() == 0.0f;
    assert adapter.getExplicitVehicles().get().get(0).getStartRot() == 90.0f;
    assert adapter.getExplicitVehicles().get().get(0).getDestZ().get() == 10.0f;

    assert adapter.getPathedVehicles().isPresent();
    assert adapter.getPathedVehicles().get().get(0).getPath().getStartX() == 123.0f;
    assert adapter.getPathedVehicles().get().get(0).getPath().getStartY() == -94.0f;
    assert adapter.getPathedVehicles().get().get(0).getPath().getDestX() == 1024.0f;
    assert adapter.getPathedVehicles().get().get(0).getPath().getDestY() == 960.0f;
    assert adapter.getPathedVehicles().get().get(0).getStartRad().getNumberUnit().equals("201.0");
    assert adapter.getPathedVehicles().get().get(0).getDestRad().getNumberUnit().equals("200.0");

    assert adapter.getRandomVehicles().isPresent();
    assert adapter.getRandomVehicles().get().size() == 2;

    assert adapter.getChannels().isPresent();
  }
}
