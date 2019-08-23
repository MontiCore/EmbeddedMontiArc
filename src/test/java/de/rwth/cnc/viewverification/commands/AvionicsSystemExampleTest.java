/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.commands;

import static org.junit.Assert.*;

import java.util.List;

import de.rwth.cnc.LogConfig;
import de.rwth.cnc.viewverification.ViewVerificator;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyItem;
import org.junit.BeforeClass;
import org.junit.Test;

public class AvionicsSystemExampleTest {

    final String TESTDIR = "src/test/resources/evalInput_edited/";

    @BeforeClass
    public static void init() {
        LogConfig.init();
    }

    @Test
    public void checkViewConnectPilotDisplayAndPCM() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.ConnectPilotDisplayAndPCM");
        //Is this test intended to not satisfy the view?
        //The fail description is wrong, if it is so.
        if (inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
                fail("Architecture does not satisfy View.");
        }
    }

    @Test
    public void checkViewControlFlowInSystem() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.ControlFlowInSystem");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkDisplayAndManager() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.DisplayAndManager");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkFlightManagerAndDirector() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.FlightManagerAndDirector");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkFlightSystemStructure() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.FlightSystemStructure");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkPilotAndPageContentManager() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.PilotAndPageContentManager");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkPilotDisplayManagerIndependentOfPilotDisplay() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.PilotDisplayManagerIndependentOfPilotDisplay");

        if (!inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkPilotDisplayManagerInsidePilotDisplay() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.PilotDisplayManagerInsidePilotDisplay");

        if (inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture does not satisfy View.");
        }
    }

    @Test
    public void checkPilotDisplayManagerPortsReversed() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.PilotDisplayManagerPortsReversed");

        if (inconsistencies.isEmpty()) {
//            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture does not satisfy View.");
        }
    }
}
