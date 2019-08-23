/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.commands;

import de.rwth.cnc.LogConfig;
import de.rwth.cnc.model.CnCArchitecture;
import de.rwth.cnc.viewverification.EmbeddedMontiArcLoader;
import org.junit.BeforeClass;
import org.junit.Test;

public class EmbeddedMontiArcLoaderTest {
    final String TESTDIR = "src/test/resources/evalInput_edited/";

    @BeforeClass
    public static void init() {
        LogConfig.init();
    }

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
