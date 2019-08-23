/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.commands;

import static org.junit.Assert.*;

import java.util.List;

import de.rwth.cnc.LogConfig;
import de.rwth.cnc.viewverification.ViewVerificator;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyItem;
import org.junit.BeforeClass;
import org.junit.Test;

public class BumperBotEmergencyTest {

    final String TESTDIR = "src/test/resources/evalInput_edited/";

    @BeforeClass
    public static void init() {
        LogConfig.init();
    }

    @Test
    public void checkViewBumperBotEmergencySystemTest() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "bumperBotEmergency.BumperBotEmergency", TESTDIR, "bumperBotEmergency.BumperBotEmergencySystem");

        if (!inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkViewMotorArbiterConnectionsEmergencyTest() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "bumperBotEmergency.BumperBotEmergency", TESTDIR, "bumperBotEmergency.MotorArbiterConnectionsEmergency");

        if (!inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkViewMotorArbiterConnectionsOldBehaviorTest() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "bumperBotEmergency.BumperBotEmergency", TESTDIR, "bumperBotEmergency.MotorArbiterConnectionsOldBehavior");

        if (!inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkViewBumperBotSensorsTest() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "bumperBotEmergency.BumperBotEmergency", TESTDIR, "bumperBotEmergency.BumperBotSensors");

        if (!inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkViewBumperBotStructureTest() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "bumperBotEmergency.BumperBotEmergency", TESTDIR, "bumperBotEmergency.BumperBotStructure");

        if (inconsistencies.isEmpty()) {
            fail("Architecture should not satisfy View.");
        }
    }

    @Test
    public void checkViewBumperBotStructureOnlyTest() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "bumperBotEmergency.BumperBotEmergency", TESTDIR, "bumperBotEmergency.BumperBotStructureOnly");

        if (!inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkViewBumperBotMotorWrongPlaceTest() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "bumperBotEmergency.BumperBotEmergency", TESTDIR, "bumperBotEmergency.BumperBotMotorWrongPlace");

        if (inconsistencies.isEmpty()) {
            fail("Architecture should not satisfy View.");
        }
    }

    @Test
    public void checkViewBumpControlOverviewTest() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "bumperBotEmergency.BumperBotEmergency", TESTDIR, "bumperBotEmergency.BumpControlOverview");

        if (!inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
    }
}
