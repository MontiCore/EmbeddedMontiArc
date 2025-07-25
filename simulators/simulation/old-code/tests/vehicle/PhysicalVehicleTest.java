/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

import simulation.EESimulator.EESimulator;
import simulation.bus.InstantBus;
import de.rwth.montisim.simulation.util.Log;

import static org.junit.Assert.assertEquals;

import java.time.Instant;

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
    public void setHeightNormal() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setHeight(2.0);
        assertEquals(2.0, physicalVehicle.getHeight(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setHeightFail() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setHeight(2.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getMassFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getMass();
    }

    @Test
    public void setMassNormal() {
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
    public void setMassFailMassPoint() {
        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setMassFailModelica() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelRadiusFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelRadius();
    }

    @Test
    public void setWheelRadiusNormal() {
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
    public void setWheelRadiusMassPoint() {
        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setWheelRadius(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelRadiusFailModelica() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setWheelRadius(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistLeftRightFrontSideFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistLeftRightFrontSide();
    }

    @Test
    public void setWheelDistLeftRightFrontSideNormal() {
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
    public void setWheelDistLeftRightFrontSideMassPoint() {
        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightFrontSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightFrontSideFailModelica() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightFrontSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistLeftRightBackSideFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistLeftRightBackSide();
    }

    @Test
    public void setWheelDistLeftRightBackSideNormal() {
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
    public void setWheelDistLeftRightBackSideMassPoint() {
        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightBackSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistLeftRightBackSideFailModelica() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setWheelDistLeftRightBackSide(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistToFrontFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistToFront();
    }

    @Test
    public void setWheelDistToFrontNormal() {
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
    public void setWheelDistToFrontMassPoint() {
        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setWheelDistToFront(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToFrontFailModelica() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setWheelDistToFront(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void getWheelDistToBackFailModelica() {
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.getWheelDistToBack();
    }

    @Test
    public void setWheelDistToBackNormal() {
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
    public void setWheelDistToBackMassPoint() {
        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setWheelDistToBack(1.0);
    }

    @Test(expected = IllegalStateException.class)
    public void setWheelDistToBackFailModelica() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setWheelDistToBack(1.0);
    }

    private Vehicle createStandardVehicle(PhysicalVehicleBuilder physicalVehicleBuilder) {
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        return new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
    }
}
