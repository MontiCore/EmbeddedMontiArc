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
