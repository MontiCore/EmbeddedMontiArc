/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;
import simulation.util.Log;

import static org.junit.Assert.assertEquals;

public class PhysicalVehicleTest {

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
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setHeight(2.0);
        assertEquals(2.0, physicalVehicle.getHeight(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setHeightFail(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setHeight(2.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getMassFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getMass();
    }

    @Test
    public void setMassNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
        assertEquals(1000.0, physicalVehicle.getMass(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setMass(1000.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1000.0, physicalVehicle.getMass(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setMassFailMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setMassFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelRadiusFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelRadius();
    }

    @Test
    public void setWheelRadiusNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setWheelRadius(1.0);
        assertEquals(1.0, physicalVehicle.getWheelRadius(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setWheelRadius(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, physicalVehicle.getWheelRadius(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelRadiusMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelRadius(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelRadiusFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelRadius(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistLeftRightFrontSideFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistLeftRightFrontSide();
    }

    @Test
    public void setWheelDistLeftRightFrontSideNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightFrontSide(1.0);
        assertEquals(1.0, physicalVehicle.getWheelDistLeftRightFrontSide(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setWheelDistLeftRightFrontSide(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, physicalVehicle.getWheelDistLeftRightFrontSide(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightFrontSideMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightFrontSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightFrontSideFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightFrontSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistLeftRightBackSideFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistLeftRightBackSide();
    }

    @Test
    public void setWheelDistLeftRightBackSideNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightBackSide(1.0);
        assertEquals(1.0, physicalVehicle.getWheelDistLeftRightBackSide(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setWheelDistLeftRightBackSide(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, physicalVehicle.getWheelDistLeftRightBackSide(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightBackSideMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightBackSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightBackSideFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightBackSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistToFrontFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistToFront();
    }

    @Test
    public void setWheelDistToFrontNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setWheelDistToFront(1.0);
        assertEquals(1.0, physicalVehicle.getWheelDistToFront(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setWheelDistToFront(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, physicalVehicle.getWheelDistToFront(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToFrontMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistToFront(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToFrontFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistToFront(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistToBackFailModelica(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistToBack();
    }

    @Test
    public void setWheelDistToBackNormal(){
        // Test MassPoint case
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setWheelDistToBack(1.0);
        assertEquals(1.0, physicalVehicle.getWheelDistToBack(), 0);

        // Test Modelica case
        ModelicaPhysicalVehicle modelicaPhysicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle = modelicaPhysicalVehicle;
        physicalVehicle.setWheelDistToBack(1.0);
        modelicaPhysicalVehicle.initPhysics();
        assertEquals(1.0, physicalVehicle.getWheelDistToBack(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToBackMassPoint(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistToBack(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToBackFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setWheelDistToBack(1.0);
    }
}
