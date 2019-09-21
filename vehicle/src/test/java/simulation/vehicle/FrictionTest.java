/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import java.time.Duration;
import java.time.Instant;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.junit.*;

import simulation.EESimulator.EESimulator;
import simulation.bus.InstantBus;
import simulation.environment.WorldModel;
import simulation.environment.visualisationadapter.interfaces.EnvStreet.StreetPavements;


public class FrictionTest {
    @Test
    public  void testCoefficientValues() {
        double frictionCoefficientDry;
        double frictionCoefficientWet;

        for(StreetPavements pavement: StreetPavements.values()) {
            frictionCoefficientDry = PhysicsEngine.calcFrictionCoefficient(pavement, false);
            frictionCoefficientWet = PhysicsEngine.calcFrictionCoefficient(pavement, true);

            try {
                // test if values are between 0 and 1
                Assert.assertTrue(0 < frictionCoefficientDry && frictionCoefficientDry < 1 && 0 < frictionCoefficientWet && frictionCoefficientWet < 1);
            } catch (AssertionError e) {
                System.out.println("At least one friction coefficient for " + pavement.toString() + " is not correctly set. Check FrictionCoefficient.csv for entries with values >1 or <= 0");
                throw e;
            }

            try {
                // test if coefficient is smaller for a wet street
                Assert.assertTrue(frictionCoefficientWet < frictionCoefficientDry);
            } catch (AssertionError e) {
                System.out.println("The friction coefficient for a dry street with pavement " + pavement.toString() + " is set to be smaller than the value for a wet pavement.");
                throw e;
            }
        }
    }

    @Test
    public void testPavementForSurface() {
		EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
		EEVehicleBuilder eeVehicleBuilder1 = new EEVehicleBuilder(eeSimulator);
		InstantBus bus1 = new InstantBus(eeSimulator);
		eeVehicleBuilder1.createAllSensorsNActuators(bus1);
    	
    	PhysicalVehicleBuilder physicalVehicleBuilder1 = new ModelicaPhysicalVehicleBuilder();
		Vehicle vehicle1 = new Vehicle(physicalVehicleBuilder1, eeVehicleBuilder1);
		ModelicaPhysicalVehicle physicalVehicle1 = (ModelicaPhysicalVehicle) vehicle1.getPhysicalVehicle();
		
		EEVehicleBuilder eeVehicleBuilder2 = new EEVehicleBuilder(eeSimulator);
		InstantBus bus2 = new InstantBus(eeSimulator);
		eeVehicleBuilder2.createAllSensorsNActuators(bus2);
		PhysicalVehicleBuilder physicalVehicleBuilder2 = new ModelicaPhysicalVehicleBuilder();
		Vehicle vehicle2 = new Vehicle(physicalVehicleBuilder2, eeVehicleBuilder2);
		ModelicaPhysicalVehicle physicalVehicle2 = (ModelicaPhysicalVehicle) vehicle2.getPhysicalVehicle();

        // puts physicalVehicle1 onto a position on the map that is known to be paved, physicalVehicle2 is already initialized with a position
        // that is known to be unpaved
        physicalVehicle1.putOnSurface(843, 236, 0.0);
        physicalVehicle1.computePhysics(Duration.ofMillis(1));
        physicalVehicle2.computePhysics(Duration.ofMillis(1));

        Assert.assertEquals(PhysicsEngine.calcFrictionCoefficient(StreetPavements.PAVED, WorldModel.getInstance().isItRaining()), physicalVehicle1.getVDM().getValue("mu_1"), 0);
        Assert.assertEquals(PhysicsEngine.calcFrictionCoefficient(StreetPavements.UNPAVED, WorldModel.getInstance().isItRaining()), physicalVehicle2.getVDM().getValue("mu_1"), 0);

        //physicalVehicle1.putOnSurface(200, 236, 0.0);

        System.out.println(physicalVehicle1);
        physicalVehicle1.computePhysics(Duration.ofMillis(4));
        System.out.println(physicalVehicle1);
    }

    public void mockSimulation() {
        RealVector expectedPosition = new ArrayRealVector(new double[]{843, 236, 0.0});
        RealVector setVelocity = new ArrayRealVector(new double[]{50.0, 0.0, 0.0});
        RealVector setAngularVelocity = new ArrayRealVector(new double[]{7.0, -2.5, 11.75});

        // Build vehicle
    	PhysicalVehicleBuilder physicalVehicleBuilder = new ModelicaPhysicalVehicleBuilder();
    	physicalVehicleBuilder.setPosition(expectedPosition);
    	physicalVehicleBuilder.setVelocity(setVelocity);
    	physicalVehicleBuilder.setAngularVelocity(setAngularVelocity);
    	
    	EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
		EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
		InstantBus bus = new InstantBus(eeSimulator);
		eeVehicleBuilder.createAllSensorsNActuators(bus);
		Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();

        physicalVehicle.computePhysics(Duration.ofMillis(1));
        System.out.println(physicalVehicle);
        System.out.println(physicalVehicle.getVDM().getValue("mu_1"));
        physicalVehicle.computePhysics(Duration.ofMillis(2000));
        System.out.println(physicalVehicle);
        System.out.println(physicalVehicle.getVDM().getValue("mu_1"));
        // TODO: Print all values to use them in a graph
    }
}
