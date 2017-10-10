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

import de.rwth.cnc.model.CnCArchitecture;
import de.rwth.cnc.viewverification.EmbeddedMontiArcLoader;
import org.junit.Test;

public class EmbeddedMontiArcLoaderTest {
  final String TESTDIR = "src/test/resources/evalInput_edited/";

  @Test
  public void loadAvionicsSystem() {
    CnCArchitecture model = EmbeddedMontiArcLoader.loadComponent(TESTDIR, "avionicsSystemExample.Flight_System");
    assert model != null : "Could not load model";
  }

  @Test
  public void loadBumperBot() {
    CnCArchitecture model = EmbeddedMontiArcLoader.loadComponent(TESTDIR, "bumperBotEmergency.BumperBotEmergency");
    assert model != null : "Could not load model";
  }

  @Test
  public void loadPumpStation() {
    CnCArchitecture model = EmbeddedMontiArcLoader.loadComponent(TESTDIR, "pumpStationExample.PumpStation");
    assert model != null : "Could not load model";
  }

  @Test
  public void loadRobotArm() {
    CnCArchitecture model = EmbeddedMontiArcLoader.loadComponent(TESTDIR, "robotArmExample.RotationalJoint");
    assert model != null : "Could not load model";
  }

  @Test
  public void loadSimpleTest() {
    CnCArchitecture model = EmbeddedMontiArcLoader.loadComponent("src/test/resources/", "simpleTests.AC");
    assert model != null : "Could not load model";
  }
}