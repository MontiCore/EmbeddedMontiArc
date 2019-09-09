/* (c) https://github.com/MontiCore/monticore */
package simulation.vehicle;

import commons.simulation.PhysicalObject;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.BeforeClass;
import org.junit.Test;
import simulation.environment.object.House;
import simulation.environment.util.VehicleType;
import simulation.util.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * JUnit test for the Vehicle class
 */
public class VehicleTest {
    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    @Test
    public void setHeightNormal(){
        Vehicle vehicle = new ModelicaPhysicalVehicle().getSimulationVehicle();
        vehicle.setHeight(2.0);
        assertEquals(2.0, vehicle.getHeight(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setHeightFail(){
        Vehicle vehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setHeight(2.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getMassFailModelica(){
        Vehicle vehicle = new ModelicaPhysicalVehicle().getSimulationVehicle();
        vehicle.getMass();
    }

    @Test
    public void setMassNormal(){
        // Test MassPoint case
        Vehicle vehicle = new MassPointPhysicalVehicle().getSimulationVehicle();
        vehicle.setMass(1000.0);
        assertEquals(1000.0, vehicle.getMass(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setMass(1000.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1000.0, vehicle.getMass(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setMassFailMassPoint(){
        Vehicle vehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setMass(1000.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setMassFailModelica() {
        Vehicle vehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setMass(1000.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelRadiusFailModelica(){
        Vehicle vehicle = new ModelicaPhysicalVehicle().getSimulationVehicle();
        vehicle.getWheelRadius();
    }

    @Test
    public void setWheelRadiusNormal(){
        // Test MassPoint case
        Vehicle vehicle = new MassPointPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelRadius(1.0);
        assertEquals(1.0, vehicle.getWheelRadius(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setWheelRadius(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, vehicle.getWheelRadius(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelRadiusMassPoint(){
        Vehicle vehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelRadius(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelRadiusFailModelica() {
        Vehicle vehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelRadius(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistLeftRightFrontSideFailModelica(){
        Vehicle vehicle = new ModelicaPhysicalVehicle().getSimulationVehicle();
        vehicle.getWheelDistLeftRightFrontSide();
    }

    @Test
    public void setWheelDistLeftRightFrontSideNormal(){
        // Test MassPoint case
        Vehicle vehicle = new MassPointPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistLeftRightFrontSide(1.0);
        assertEquals(1.0, vehicle.getWheelDistLeftRightFrontSide(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setWheelDistLeftRightFrontSide(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, vehicle.getWheelDistLeftRightFrontSide(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightFrontSideMassPoint(){
        Vehicle vehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistLeftRightFrontSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightFrontSideFailModelica() {
        Vehicle vehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistLeftRightFrontSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistLeftRightBackSideFailModelica(){
        Vehicle vehicle = new ModelicaPhysicalVehicle().getSimulationVehicle();
        vehicle.getWheelDistLeftRightBackSide();
    }

    @Test
    public void setWheelDistLeftRightBackSideNormal(){
        // Test MassPoint case
        Vehicle vehicle = new MassPointPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistLeftRightBackSide(1.0);
        assertEquals(1.0, vehicle.getWheelDistLeftRightBackSide(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setWheelDistLeftRightBackSide(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, vehicle.getWheelDistLeftRightBackSide(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightBackSideMassPoint(){
        Vehicle vehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistLeftRightBackSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightBackSideFailModelica() {
        Vehicle vehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistLeftRightBackSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistToFrontFailModelica(){
        Vehicle vehicle = new ModelicaPhysicalVehicle().getSimulationVehicle();
        vehicle.getWheelDistToFront();
    }

    @Test
    public void setWheelDistToFrontNormal(){
        // Test MassPoint case
        Vehicle vehicle = new MassPointPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistToFront(1.0);
        assertEquals(1.0, vehicle.getWheelDistToFront(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setWheelDistToFront(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, vehicle.getWheelDistToFront(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToFrontMassPoint(){
        Vehicle vehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistToFront(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToFrontFailModelica() {
        Vehicle vehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistToFront(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistToBackFailModelica(){
        Vehicle vehicle = new ModelicaPhysicalVehicle().getSimulationVehicle();
        vehicle.getWheelDistToBack();
    }

    @Test
    public void setWheelDistToBackNormal(){
        // Test MassPoint case
        Vehicle vehicle = new MassPointPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistToBack(1.0);
        assertEquals(1.0, vehicle.getWheelDistToBack(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setWheelDistToBack(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, vehicle.getWheelDistToBack(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToBackMassPoint(){
        Vehicle vehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistToBack(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToBackFailModelica() {
        Vehicle vehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle().getSimulationVehicle();
        vehicle.setWheelDistToBack(1.0);
    }

    @Test
    public void setVehicleInitialisedTrue(){
        Vehicle vehicle = new ModelicaPhysicalVehicle().getSimulationVehicle();
        vehicle.setVehicleInitialised(true);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setVehicleInitialisedFail(){
        Vehicle vehicle = new ModelicaPhysicalVehicle().getSimulationVehicle();
        vehicle.setVehicleInitialised(false);
    }

    @Test
    public void computePhysicsTest() {
        long timeDiffMs = 10;

        // 2 colliding vehicles
        PhysicalObject vehicle1 = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        PhysicalObject vehicle2 = new MassPointPhysicalVehicleBuilder().setPosition(new ArrayRealVector(new double[] {1, 0, 0})).buildPhysicalVehicle();
        List<PhysicalObject> physicalObjects = new ArrayList<>();
        physicalObjects.add(vehicle2);
        PhysicsEngine.computePhysics(vehicle1, physicalObjects, timeDiffMs);
        assertTrue(vehicle1.getCollision() && vehicle2.getCollision());

        // 2 non colliding vehicles
        physicalObjects.clear();
        vehicle1 = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        vehicle2 = new MassPointPhysicalVehicleBuilder().setPosition(new ArrayRealVector(new double[] {10, 0, 0})).buildPhysicalVehicle();
        physicalObjects.add(vehicle2);
        PhysicsEngine.computePhysics(vehicle1, physicalObjects, timeDiffMs);
        assertTrue(!vehicle1.getCollision() && !vehicle2.getCollision());

        // Vehicle colliding with non vehicle
        physicalObjects.clear();
        vehicle1 = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        vehicle2 = new MassPointPhysicalVehicleBuilder().setPosition(new ArrayRealVector(new double[] {30, 0, 0})).buildPhysicalVehicle();
        House house = new House();
        house.setPosition(new ArrayRealVector(new double[] {1, 0, 0}));
        house.setWidth(10.0);
        house.setLength(10.0);
        house.setHeight(10.0);
        physicalObjects.add(vehicle2);
        physicalObjects.add(house);
        PhysicsEngine.computePhysics(vehicle1, physicalObjects, timeDiffMs);
        assertTrue(vehicle1.getCollision() && !vehicle2.getCollision());
    }

    @Test
    public void AutopilotBehaviorTest() {
        //Masspoint
        Vehicle vehicle = new MassPointPhysicalVehicle(VehicleType.ELECTRIC,0).getSimulationVehicle();
        vehicle.checkBattery();
        assertTrue(vehicle.isGotoCharginstation());
        assertTrue(vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR).getActuatorValueCurrent()==0);
        Vehicle vehicle2 = new MassPointPhysicalVehicle(VehicleType.ELECTRIC,30).getSimulationVehicle();
        vehicle2.checkBattery();
        //assertTrue(vehicle2.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR).getActuatorValueCurrent()!=0);
        assertTrue(!vehicle2.isGotoCharginstation());
        //Modelica
        Vehicle vehicle3 = new ModelicaPhysicalVehicle(VehicleType.ELECTRIC,0).getSimulationVehicle();
        vehicle3.checkBattery();
        assertTrue(vehicle3.isGotoCharginstation());
        Vehicle vehicle4 = new ModelicaPhysicalVehicle(VehicleType.ELECTRIC,30).getSimulationVehicle();
        vehicle4.checkBattery();
        assertTrue(!vehicle4.isGotoCharginstation());
    }
}
