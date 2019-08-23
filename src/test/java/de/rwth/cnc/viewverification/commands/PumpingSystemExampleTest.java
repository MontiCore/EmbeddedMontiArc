/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.commands;

import de.rwth.cnc.LogConfig;
import de.rwth.cnc.viewverification.ViewVerificator;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyItem;
import org.junit.BeforeClass;
import org.junit.Test;

import static org.junit.Assert.*;

import java.util.List;

public class PumpingSystemExampleTest {

    final String TESTDIR = "src/test/resources/evalInput_edited/";
    final String message = "failed.";

    @BeforeClass
    public static void init() {
        LogConfig.init();
    }

    @Test
    public void checkViewASPumpingSystem() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.ASPumpingSystem");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail(message);
        }
    }

    @Test
    public void checkViewEnvironmentPhysics() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.EnvironmentPhysics");

        if (!inconsistencies.isEmpty()) {
            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail(message); //TODO this test fails since autoconnect doesnt work!
        }
    }

    @Test
    public void checkModeArbiterOutsideController() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.ModeArbiterOutsideController");

        if (inconsistencies.isEmpty()) {
            fail("Architecture does not satisfy View.");
        }
    }

    @Test
    public void checkPhysicsAndControllerPumpingSystem() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.PhysicsAndControllerPumpingSystem");

        if (inconsistencies.isEmpty()) {
            fail("Architecture does not satisfy View.");
        }
    }

    @Test
    public void checkPhysicsInsidePumpingSystem() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.PhysicsInsidePumpingSystem");

        if (inconsistencies.isEmpty()) {
            fail("Architecture does not satisfy View.");
        }
    }

    @Test
    public void checkPumpStationStructure() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.PumpStationStructure");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail(message);
        }
    }

    @Test
    public void checkPumpingSystemStructure() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.PumpingSystemStructure");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail(message);
        }
    }

    @Test
    public void checkSimulationInput() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.SimulationInput");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail(message);
        }
    }

    @Test
    public void checkSystemEmergencyController() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.SystemEmergencyController");

        if (inconsistencies.isEmpty()) {
            fail("Architecture does not satisfy View.");
        }
    }

    @Test
    public void checkSystemEmergencyControllerFixed() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.SystemEmergencyControllerFixed");

        if (inconsistencies.isEmpty()) {
            fail("Architecture does not satisfy View.");
        }
    }

    @Test
    public void checkUserButton() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.UserButton");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail(message);
        }
    }

    @Test
    public void checkPositiveEffector() {

        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.PositiveEffector");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail(message);
        }
    }

    //  private String getMessageWithReasonsForInconsistency(ConsistencyCommandMontiArc cmd) {
    //    StringBuffer buf = new StringBuffer();
    //    for (InconsistencyItem item : cmd.getResult()) {
    //      buf.append(item.getJustificationDescription() + " ");
    //    }
    //    String message = "Architecture should satisfy View. But: " + buf.toString();
    //    return message;
    //  }
}
