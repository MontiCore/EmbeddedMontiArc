/**
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.rwth.cnc.viewverification.commands;

import de.rwth.cnc.viewverification.ViewVerificator;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyItem;
import org.junit.Test;

import static org.junit.Assert.*;

import java.util.List;

public class PumpingSystemExampleTest {

  final String TESTDIR = "src/test/resources/evalInput_edited/";
  final String message = "failed.";

  @Test
  public void checkViewASPumpingSystem() {

    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.ASPumpingSystem");

    if (!inconsistencies.isEmpty()) {
      inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
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
      inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
      fail(message);
    }
  }

  @Test
  public void checkPumpingSystemStructure() {

    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.PumpingSystemStructure");

    if (!inconsistencies.isEmpty()) {
      inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
      fail(message);
    }
  }

  @Test
  public void checkSimulationInput() {

    List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "pumpStationExample.PumpStation", TESTDIR, "pumpStationExample.SimulationInput");

    if (!inconsistencies.isEmpty()) {
      inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
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
      inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
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
