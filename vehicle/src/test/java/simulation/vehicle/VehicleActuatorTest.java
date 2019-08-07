/**
 *
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
package simulation.vehicle;

import commons.controller.commons.BusEntry;
import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.util.Log;

import java.time.Instant;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

/**
 * Class that tests the VehicleActuator class
 */
public class VehicleActuatorTest {
    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    @Test
    public void vehicleActuatorNormal() {
        // Test normal case
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);

        Assert.assertEquals(0, actuator.getActuatorValueMin(), 0);
        Assert.assertEquals(10, actuator.getActuatorValueMax(), 0);
        Assert.assertEquals(1, actuator.getActuatorValueChangeRate(), 0);

        Assert.assertEquals(actuator.getActuatorValueCurrent(), actuator.getActuatorValueTarget(), 0);
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueCurrent());
        Assert.assertTrue(actuator.getActuatorValueCurrent() <= actuator.getActuatorValueMax());
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueTarget());
        Assert.assertTrue(actuator.getActuatorValueTarget() <= actuator.getActuatorValueMax());

        // Test non zero case
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 1, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);

        Assert.assertEquals(1, actuator.getActuatorValueMin(), 0);
        Assert.assertEquals(10, actuator.getActuatorValueMax(), 0);
        Assert.assertEquals(1, actuator.getActuatorValueChangeRate(), 0);

        Assert.assertEquals(actuator.getActuatorValueCurrent(), actuator.getActuatorValueTarget(), 0);
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueCurrent());
        Assert.assertTrue(actuator.getActuatorValueCurrent() <= actuator.getActuatorValueMax());
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueTarget());
        Assert.assertTrue(actuator.getActuatorValueTarget() <= actuator.getActuatorValueMax());

        // Test all negative case
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, -10, -1, 1, simulator, Collections.emptyList(), targetsByMessageId);

        Assert.assertEquals(-10, actuator.getActuatorValueMin(), 0);
        Assert.assertEquals(-1, actuator.getActuatorValueMax(), 0);
        Assert.assertEquals(1, actuator.getActuatorValueChangeRate(), 0);

        Assert.assertEquals(actuator.getActuatorValueCurrent(), actuator.getActuatorValueTarget(), 0);
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueCurrent());
        Assert.assertTrue(actuator.getActuatorValueCurrent() <= actuator.getActuatorValueMax());
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueTarget());
        Assert.assertTrue(actuator.getActuatorValueTarget() <= actuator.getActuatorValueMax());

        // Test min == max case
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 10, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);

        Assert.assertEquals(10, actuator.getActuatorValueMin(), 0);
        Assert.assertEquals(10, actuator.getActuatorValueMax(), 0);
        Assert.assertEquals(1, actuator.getActuatorValueChangeRate(), 0);

        Assert.assertEquals(actuator.getActuatorValueCurrent(), actuator.getActuatorValueTarget(), 0);
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueCurrent());
        Assert.assertTrue(actuator.getActuatorValueCurrent() <= actuator.getActuatorValueMax());
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueTarget());
        Assert.assertTrue(actuator.getActuatorValueTarget() <= actuator.getActuatorValueMax());
    }

    @Test(expected = IllegalArgumentException.class)
    public void vehicleActuatorFailMinMax() {
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 10, 0, 1, simulator, Collections.emptyList(), targetsByMessageId);
    }

    @Test(expected = IllegalArgumentException.class)
    public void vehicleActuatorFailRate(){
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, -1, simulator, Collections.emptyList(), targetsByMessageId);
    }

    @Test
    public void updateTest(){
        // Test approach target
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueTarget(5.0);
        actuator.setActuatorValueCurrent(0.0);
        actuator.update(1);
        Assert.assertEquals(5.0, actuator.getActuatorValueTarget(), 0);
        Assert.assertEquals(1.0, actuator.getActuatorValueCurrent(), 0);

        // Test reaching target
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueTarget(5.0);
        actuator.setActuatorValueCurrent(4.0);
        actuator.update(1);
        Assert.assertEquals(5.0, actuator.getActuatorValueTarget(), 0);
        Assert.assertEquals(5.0, actuator.getActuatorValueCurrent(), 0);

        // Test not overshooting
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueTarget(5.0);
        actuator.setActuatorValueCurrent(4.5);
        actuator.update(1);
        Assert.assertEquals(5.0, actuator.getActuatorValueTarget(), 0);
        Assert.assertEquals(5.0, actuator.getActuatorValueCurrent(), 0);

        // Test already reached target
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueTarget(5.0);
        actuator.setActuatorValueCurrent(5.0);
        actuator.update(1);
        Assert.assertEquals(5.0, actuator.getActuatorValueTarget(), 0);
        Assert.assertEquals(5.0, actuator.getActuatorValueCurrent(), 0);
    }

    @Test
    public void setActuatorValueTargetNormal(){
        // Test normal case
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueTarget(5.0);
        Assert.assertEquals(5.0, actuator.getActuatorValueTarget(), 0);

        // Test setting on max value
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueTarget(10.0);
        Assert.assertEquals(10.0, actuator.getActuatorValueTarget(), 0);

        // Test setting on min value
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueTarget(0.0);
        Assert.assertEquals(0.0, actuator.getActuatorValueTarget(), 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setActuatorValueTargetOverMax() {
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueTarget(11.0);
        Assert.assertTrue(true);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setActuatorValueTargetUnderMin(){
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueTarget(-1.0);
    }

    @Test
    public void setActuatorValueCurrentNormal(){
        // Test normal case
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueCurrent(5.0);
        Assert.assertEquals(5.0, actuator.getActuatorValueCurrent(), 0);

        // Test setting on max value
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueCurrent(10.0);
        Assert.assertEquals(10.0, actuator.getActuatorValueCurrent(), 0);

        // Test setting on min value
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueCurrent(0.0);
        Assert.assertEquals(0.0, actuator.getActuatorValueCurrent(), 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setActuatorValueCurrentOverMax() {
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueCurrent(11.0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setActuatorValueCurrentUnderMin(){
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1, simulator, Collections.emptyList(), targetsByMessageId);
        actuator.setActuatorValueCurrent(-1.0);
    }
}