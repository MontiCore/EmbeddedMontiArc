package simulation.vehicle;

import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;
import simulation.util.Log;

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
        Assert.assertEquals(2.0, vehicle.getHeight(), 0);
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
        Assert.assertEquals(1000.0, vehicle.getMass(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setMass(1000.0);
        modelicaPhysicalVehicle.initPhysics();
        Assert.assertEquals(1000.0, vehicle.getMass(), 0);
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
        Assert.assertEquals(1.0, vehicle.getWheelRadius(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setWheelRadius(1.0);
        modelicaPhysicalVehicle.initPhysics();
        Assert.assertEquals(1.0, vehicle.getWheelRadius(), 0);
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
        Assert.assertEquals(1.0, vehicle.getWheelDistLeftRightFrontSide(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setWheelDistLeftRightFrontSide(1.0);
        modelicaPhysicalVehicle.initPhysics();
        Assert.assertEquals(1.0, vehicle.getWheelDistLeftRightFrontSide(), 0);
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
        Assert.assertEquals(1.0, vehicle.getWheelDistLeftRightBackSide(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setWheelDistLeftRightBackSide(1.0);
        modelicaPhysicalVehicle.initPhysics();
        Assert.assertEquals(1.0, vehicle.getWheelDistLeftRightBackSide(), 0);
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
        Assert.assertEquals(1.0, vehicle.getWheelDistToFront(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setWheelDistToFront(1.0);
        modelicaPhysicalVehicle.initPhysics();
        Assert.assertEquals(1.0, vehicle.getWheelDistToFront(), 0);
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
        Assert.assertEquals(1.0, vehicle.getWheelDistToBack(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        vehicle = modelicaPhysicalVehicle.getSimulationVehicle();
        vehicle.setWheelDistToBack(1.0);
        modelicaPhysicalVehicle.initPhysics();
        Assert.assertEquals(1.0, vehicle.getWheelDistToBack(), 0);
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
    public void setVehicleInitializedTrue(){
        Vehicle vehicle = new ModelicaPhysicalVehicle().getSimulationVehicle();
        vehicle.setVehicleInitialized(true);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setVehicleInitializedFail(){
        Vehicle vehicle = new ModelicaPhysicalVehicle().getSimulationVehicle();
        vehicle.setVehicleInitialized(false);
    }
}