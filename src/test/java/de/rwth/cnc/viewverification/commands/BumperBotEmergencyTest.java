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
