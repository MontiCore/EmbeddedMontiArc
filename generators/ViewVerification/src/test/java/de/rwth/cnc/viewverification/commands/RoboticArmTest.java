/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.commands;

import static org.junit.Assert.*;

import java.util.List;

import de.rwth.cnc.LogConfig;
import de.rwth.cnc.viewverification.ViewVerificator;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyItem;
import org.junit.BeforeClass;
import org.junit.Test;


public class RoboticArmTest {

    final String TESTDIR = "src/test/resources/evalInput_edited/";

    @BeforeClass
    public static void init() {
        LogConfig.init();
    }

    @Test
    public void ASDependence() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "robotArmExample.RotationalJoint", TESTDIR, "robotArmExample.ASDependence");


//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        if (inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
        System.out.println("Test successful");
    }

    @Test
    public void BodySensorIn() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "robotArmExample.RotationalJoint", TESTDIR, "robotArmExample.BodySensorIn");

//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        if (inconsistencies.isEmpty()) {
            fail("Architecture should not satisfy View.");
        }
        System.out.println("Test successful");
    }

    @Test
    public void BodySensorOut() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "robotArmExample.RotationalJoint", TESTDIR, "robotArmExample.BodySensorOut");

//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        if (inconsistencies.isEmpty()) {
            fail("Architecture should not satisfy View.");
        }
        System.out.println("Test successful");
    }

    @Test
    public void RJFunction() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "robotArmExample.RotationalJoint", TESTDIR, "robotArmExample.RJFunction");

//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        if (inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
        System.out.println("Test successful");
    }

    @Test
    public void OldDesign() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.
                verify(TESTDIR, "robotArmExample.RotationalJoint", TESTDIR, "robotArmExample.OldDesign");

//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        if (inconsistencies.isEmpty()) {
            fail("Architecture should not satisfy View.");
        }
        System.out.println("Test successful");
    }

    @Test
    public void OldDesignExternalCylinder() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "robotArmExample.RotationalJoint", TESTDIR, "robotArmExample.OldDesignExternalCylinder");

//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        if (inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
        System.out.println("Test successful");
    }

    @Test
    public void SensorAmplifierView() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "robotArmExample.RotationalJoint", TESTDIR, "robotArmExample.SensorAmplifierView");

//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        if (inconsistencies.isEmpty()) {
            fail("Architecture should not satisfy View.");
        }
        System.out.println("Test successful");
    }

    @Test
    public void SensorConnections() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "robotArmExample.RotationalJoint", TESTDIR, "robotArmExample.SensorConnections");

//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        if (inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
        System.out.println("Test successful");
    }

    @Test
    public void SensorConnectionsInterfaceComplete() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "robotArmExample.RotationalJoint", TESTDIR, "robotArmExample.SensorConnectionsInterfaceComplete");

//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        if (inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
        System.out.println("Test successful");
    }

    @Test
    public void SensorHasAmplifier() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "robotArmExample.RotationalJoint", TESTDIR, "robotArmExample.SensorHasAmplifier");

//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        if (inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
        System.out.println("Test successful");
    }

    @Test
    public void RJStructure() {
        List<InconsistencyItem> inconsistencies = ViewVerificator.verify(TESTDIR, "robotArmExample.RotationalJoint", TESTDIR, "robotArmExample.RJStructure");

//        inconsistencies.forEach(c -> System.out.println(c.getJustificationDescription()));
        if (inconsistencies.isEmpty()) {
            fail("Architecture should satisfy View.");
        }
        System.out.println("Test successful");
    }
}
