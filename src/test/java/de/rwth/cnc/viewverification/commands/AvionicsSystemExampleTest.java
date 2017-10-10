/**
 * ******************************************************************************
 * MontiCAR Modeling Family, www.se-rwth.de
 * Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 * All rights reserved.
 * <p>
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * <p>
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture does not satisfy View.");
        }
    }

    @Test
    public void checkViewControlFlowInSystem() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.ControlFlowInSystem");

        if (!inconsistencies.isEmpty()) {
            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkDisplayAndManager() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.DisplayAndManager");

        if (!inconsistencies.isEmpty()) {
            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkFlightManagerAndDirector() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.FlightManagerAndDirector");

        if (!inconsistencies.isEmpty()) {
            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkFlightSystemStructure() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.FlightSystemStructure");

        if (!inconsistencies.isEmpty()) {
            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkPilotAndPageContentManager() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.PilotAndPageContentManager");

        if (!inconsistencies.isEmpty()) {
            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkPilotDisplayManagerIndependentOfPilotDisplay() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.PilotDisplayManagerIndependentOfPilotDisplay");

        if (!inconsistencies.isEmpty()) {
            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture should satisfy View.");
        }
    }

    @Test
    public void checkPilotDisplayManagerInsidePilotDisplay() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.PilotDisplayManagerInsidePilotDisplay");

        if (inconsistencies.isEmpty()) {
            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture does not satisfy View.");
        }
    }

    @Test
    public void checkPilotDisplayManagerPortsReversed() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "avionicsSystemExample.Flight_System", TESTDIR, "avionicsSystemExample.PilotDisplayManagerPortsReversed");

        if (inconsistencies.isEmpty()) {
            inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
            fail("Architecture does not satisfy View.");
        }
    }
}