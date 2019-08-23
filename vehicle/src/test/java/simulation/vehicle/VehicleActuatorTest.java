/* (c) https://github.com/MontiCore/monticore */
package simulation.vehicle;

import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;
import simulation.util.Log;

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
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);

        Assert.assertEquals(0, actuator.getActuatorValueMin(), 0);
        Assert.assertEquals(10, actuator.getActuatorValueMax(), 0);
        Assert.assertEquals(1, actuator.getActuatorValueChangeRate(), 0);

        Assert.assertEquals(actuator.getActuatorValueCurrent(), actuator.getActuatorValueTarget(), 0);
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueCurrent());
        Assert.assertTrue(actuator.getActuatorValueCurrent() <= actuator.getActuatorValueMax());
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueTarget());
        Assert.assertTrue(actuator.getActuatorValueTarget() <= actuator.getActuatorValueMax());

        // Test non zero case
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 1, 10, 1);

        Assert.assertEquals(1, actuator.getActuatorValueMin(), 0);
        Assert.assertEquals(10, actuator.getActuatorValueMax(), 0);
        Assert.assertEquals(1, actuator.getActuatorValueChangeRate(), 0);

        Assert.assertEquals(actuator.getActuatorValueCurrent(), actuator.getActuatorValueTarget(), 0);
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueCurrent());
        Assert.assertTrue(actuator.getActuatorValueCurrent() <= actuator.getActuatorValueMax());
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueTarget());
        Assert.assertTrue(actuator.getActuatorValueTarget() <= actuator.getActuatorValueMax());

        // Test all negative case
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, -10, -1, 1);

        Assert.assertEquals(-10, actuator.getActuatorValueMin(), 0);
        Assert.assertEquals(-1, actuator.getActuatorValueMax(), 0);
        Assert.assertEquals(1, actuator.getActuatorValueChangeRate(), 0);

        Assert.assertEquals(actuator.getActuatorValueCurrent(), actuator.getActuatorValueTarget(), 0);
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueCurrent());
        Assert.assertTrue(actuator.getActuatorValueCurrent() <= actuator.getActuatorValueMax());
        Assert.assertTrue(actuator.getActuatorValueMin() <= actuator.getActuatorValueTarget());
        Assert.assertTrue(actuator.getActuatorValueTarget() <= actuator.getActuatorValueMax());

        // Test min == max case
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 10, 10, 1);

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
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 10, 0, 1);
    }

    @Test(expected = IllegalArgumentException.class)
    public void vehicleActuatorFailRate(){
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, -1);
    }

    @Test
    public void updateTest(){
        // Test approach target
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueTarget(5.0);
        actuator.setActuatorValueCurrent(0.0);
        actuator.update(1);
        Assert.assertEquals(5.0, actuator.getActuatorValueTarget(), 0);
        Assert.assertEquals(1.0, actuator.getActuatorValueCurrent(), 0);

        // Test reaching target
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueTarget(5.0);
        actuator.setActuatorValueCurrent(4.0);
        actuator.update(1);
        Assert.assertEquals(5.0, actuator.getActuatorValueTarget(), 0);
        Assert.assertEquals(5.0, actuator.getActuatorValueCurrent(), 0);

        // Test not overshooting
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueTarget(5.0);
        actuator.setActuatorValueCurrent(4.5);
        actuator.update(1);
        Assert.assertEquals(5.0, actuator.getActuatorValueTarget(), 0);
        Assert.assertEquals(5.0, actuator.getActuatorValueCurrent(), 0);

        // Test already reached target
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueTarget(5.0);
        actuator.setActuatorValueCurrent(5.0);
        actuator.update(1);
        Assert.assertEquals(5.0, actuator.getActuatorValueTarget(), 0);
        Assert.assertEquals(5.0, actuator.getActuatorValueCurrent(), 0);
    }

    @Test
    public void setActuatorValueTargetNormal(){
        // Test normal case
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueTarget(5.0);
        Assert.assertEquals(5.0, actuator.getActuatorValueTarget(), 0);

        // Test setting on max value
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueTarget(10.0);
        Assert.assertEquals(10.0, actuator.getActuatorValueTarget(), 0);

        // Test setting on min value
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueTarget(0.0);
        Assert.assertEquals(0.0, actuator.getActuatorValueTarget(), 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setActuatorValueTargetOverMax() {
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueTarget(11.0);
        Assert.assertTrue(true);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setActuatorValueTargetUnderMin(){
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueTarget(-1.0);
    }

    @Test
    public void setActuatorValueCurrentNormal(){
        // Test normal case
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueCurrent(5.0);
        Assert.assertEquals(5.0, actuator.getActuatorValueCurrent(), 0);

        // Test setting on max value
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueCurrent(10.0);
        Assert.assertEquals(10.0, actuator.getActuatorValueCurrent(), 0);

        // Test setting on min value
        actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueCurrent(0.0);
        Assert.assertEquals(0.0, actuator.getActuatorValueCurrent(), 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setActuatorValueCurrentOverMax() {
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueCurrent(11.0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setActuatorValueCurrentUnderMin(){
        VehicleActuator actuator = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, 0, 10, 1);
        actuator.setActuatorValueCurrent(-1.0);
    }
}
