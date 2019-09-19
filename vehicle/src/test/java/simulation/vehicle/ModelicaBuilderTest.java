/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.interfaces.Bus;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.interfaces.FunctionBlockInterface;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.PhysicalObjectType;
import de.rwth.monticore.EmbeddedMontiArc.simulators.controller.navigation.navigationBlock.NavigationBlock;

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
import simulation.util.Log;
import simulation.util.MathHelper;

import java.time.Instant;
import java.util.Optional;

/**
 * Class that tests the ModelicaPhysicalVehicleBuilder class
 */
public class ModelicaBuilderTest {
    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    @Test
    public void buildDefaultVehicle(){
        // Get default VDM values
    	Vehicle referenceVehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle referencePhysicalVehicle = (ModelicaPhysicalVehicle) referenceVehicle.getPhysicalVehicle();
        VehicleDynamicsModel referenceVDM = referencePhysicalVehicle.getVDM();

        // Calculate expected values
        double z = referenceVDM.getValue("z");
        double height = PhysicalVehicle.VEHICLE_DEFAULT_HEIGHT;
        RealVector expectedPosition = new ArrayRealVector(new Double[]{0.0, 0.0, - height/2 + z});
        Rotation expectedRot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, 0.0);
        RealMatrix expectedRotation = new BlockRealMatrix(expectedRot.getMatrix());
        RealVector expectedVelocity = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        RealVector expectedAngularVelocity = new ArrayRealVector(3);
        double expectedMass = referenceVDM.getValue("m");
        double expectedWidth = PhysicalVehicle.VEHICLE_DEFAULT_WIDTH;
        double expectedLength = PhysicalVehicle.VEHICLE_DEFAULT_LENGTH;
        double expectedHeight = PhysicalVehicle.VEHICLE_DEFAULT_HEIGHT;
        double expectedWheelRadius = referenceVDM.getValue("r_nom");
        double expectedWheelDistLeftRightFrontSide = referenceVDM.getValue("TW_f");
        double expectedWheelDistLeftRightBackSide = referenceVDM.getValue("TW_r");
        double expectedWheelDistToFront = referenceVDM.getValue("L_1");
        double expectedWheelDistToBack = referenceVDM.getValue("L_2");

        // Calculate expected remaining values
        RealVector expectedForce = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        RealVector expectedGeometryPosition = new ArrayRealVector(new Double[]{0.0, 0.0, 0.0});
        double expectedWheelRotationRate = 0.0;

        // Build default car
        ModelicaPhysicalVehicleBuilder builder = new ModelicaPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(builder);
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();

        // Test default/not set parameters
        Assert.assertTrue(MathHelper.vectorEquals(expectedPosition, physicalVehicle.getPosition(), 0.00000001));
        Assert.assertTrue(MathHelper.matrixEquals(expectedRotation, physicalVehicle.getRotation(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedVelocity, physicalVehicle.getVelocity(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedAngularVelocity, physicalVehicle.getAngularVelocity(), 0.00000001));
        Assert.assertEquals(expectedMass, physicalVehicle.getMass(), 0);
        Assert.assertEquals(expectedWidth, physicalVehicle.getWidth(), 0);
        Assert.assertEquals(expectedLength, physicalVehicle.getLength(), 0);
        Assert.assertEquals(expectedHeight, physicalVehicle.getHeight(), 0);
        Assert.assertEquals(expectedWheelRadius, physicalVehicle.getWheelRadius(), 0);
        Assert.assertEquals(expectedWheelDistLeftRightFrontSide, physicalVehicle.getWheelDistLeftRightFrontSide(), 0);
        Assert.assertEquals(expectedWheelDistLeftRightBackSide, physicalVehicle.getWheelDistLeftRightBackSide(), 0);
        Assert.assertEquals(expectedWheelDistToFront, physicalVehicle.getWheelDistToFront(), 0);
        Assert.assertEquals(expectedWheelDistToBack, physicalVehicle.getWheelDistToBack(), 0);

        // Test internal values
        Assert.assertEquals(PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR, physicalVehicle.getPhysicalObjectType());
        Assert.assertFalse(physicalVehicle.getError());
        Assert.assertFalse(physicalVehicle.getCollision());
        Assert.assertTrue(physicalVehicle.getPhysicalVehicleInitialised());

        // Test physical values
        Assert.assertTrue(MathHelper.vectorEquals(expectedForce, physicalVehicle.getForce(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedGeometryPosition, physicalVehicle.getGeometryPosition(), 0.00000001));
        Assert.assertEquals(expectedWheelRotationRate, physicalVehicle.getVDM().getValue("omega_wheel_1"), 0);
        Assert.assertEquals(expectedWheelRotationRate, physicalVehicle.getVDM().getValue("omega_wheel_2"), 0);
        Assert.assertEquals(expectedWheelRotationRate, physicalVehicle.getVDM().getValue("omega_wheel_3"), 0);
        Assert.assertEquals(expectedWheelRotationRate, physicalVehicle.getVDM().getValue("omega_wheel_4"), 0);
    }

    @Test
    public void buildCustomVehicle(){
        // Get default VDM values
        ModelicaPhysicalVehicleBuilder modelicaBuilder = new ModelicaPhysicalVehicleBuilder();
        Vehicle referenceVehicle = createStandardVehicle(modelicaBuilder);
        ModelicaPhysicalVehicle referencePhysicalVehicle = (ModelicaPhysicalVehicle) referenceVehicle.getPhysicalVehicle();
        VehicleDynamicsModel referenceVDM = referencePhysicalVehicle.getVDM();

        // Calculate expected values
        double z = referenceVDM.getValue("z");
        RealVector expectedPosition = new ArrayRealVector(new double[]{12.0, -3.0, 3.5});
        Rotation expectedRot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 7.0, -3.2, Math.PI / 2);
        RealMatrix expectedRotation = new BlockRealMatrix(expectedRot.getMatrix());
        RealMatrix internalRotation = ModelicaPhysicalVehicle.coordinateRotation.multiply(expectedRotation);
        RealVector setVelocity = new ArrayRealVector(new double[]{1.0, -5.725, 12.0});
        RealVector setVelocityLocal = internalRotation.transpose().operate(setVelocity);
        RealVector expectedVelocityLocal = new ArrayRealVector(new double[]{setVelocityLocal.getEntry(0), setVelocityLocal.getEntry(1), 0.0});
        RealVector expectedVelocity = internalRotation.operate(expectedVelocityLocal);
        RealVector setAngularVelocity = new ArrayRealVector(new double[]{7.0, -2.5, 11.75});
        RealVector setAngularVelocityLocal = internalRotation.transpose().operate(setAngularVelocity);
        RealVector expectedAngularVelocityLocal = new ArrayRealVector(new double[]{0.0, 0.0, setAngularVelocityLocal.getEntry(2)});
        RealVector expectedAngularVelocity = internalRotation.operate(expectedAngularVelocityLocal);
        double expectedMass = 1000.0;
        double expectedWidth = 2.125;
        double expectedLength = 10.0;
        double expectedHeight = 3.5;
        double expectedWheelRadius = 0.66;
        double expectedWheelDistLeftRightFrontSide = 1.1;
        double expectedWheelDistLeftRightBackSide = 2;
        double expectedWheelDistToFront = 2.75;
        double expectedWheelDistToBack = 4.25;

        // Calculate remaining expected values
        RealVector expectedForce = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        RealVector expectedGeometryPositionOffset = internalRotation.operate(new ArrayRealVector(new double[]{0.0, 0.0, expectedHeight/2 - z}));
        RealVector expectedGeometryPosition = expectedPosition.add(expectedGeometryPositionOffset);
        double expectedWheelRotationRate = expectedVelocityLocal.getEntry(0) / expectedWheelRadius;

        // Build custom car
        ModelicaPhysicalVehicleBuilder builder = new ModelicaPhysicalVehicleBuilder();
        builder.setPosition(expectedPosition);
        builder.setRotation(expectedRot);
        builder.setVelocity(setVelocity);
        builder.setAngularVelocity(setAngularVelocity);
        builder.setMass(expectedMass);
        builder.setWidth(expectedWidth);
        builder.setLength(expectedLength);
        builder.setHeight(expectedHeight);
        builder.setWheelRadius(expectedWheelRadius);
        builder.setWheelDistLeftRightFrontSide(expectedWheelDistLeftRightFrontSide);
        builder.setWheelDistLeftRightBackSide(expectedWheelDistLeftRightBackSide);
        builder.setWheelDistToFront(expectedWheelDistToFront);
        builder.setWheelDistToBack(expectedWheelDistToBack);
        Vehicle vehicle = createStandardVehicle(builder);
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();

        // Test custom set parameters
        Assert.assertTrue(MathHelper.vectorEquals(expectedPosition, physicalVehicle.getPosition(), 0.00000001));
        Assert.assertTrue(MathHelper.matrixEquals(expectedRotation, physicalVehicle.getRotation(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedVelocity, physicalVehicle.getVelocity(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedAngularVelocity, physicalVehicle.getAngularVelocity(), 0.00000001));
        Assert.assertEquals(expectedMass, physicalVehicle.getMass(), 0);
        Assert.assertEquals(expectedWidth, physicalVehicle.getWidth(), 0);
        Assert.assertEquals(expectedLength, physicalVehicle.getLength(), 0);
        Assert.assertEquals(expectedHeight, physicalVehicle.getHeight(), 0);
        Assert.assertEquals(expectedWheelRadius, physicalVehicle.getWheelRadius(), 0);
        Assert.assertEquals(expectedWheelDistLeftRightFrontSide, physicalVehicle.getWheelDistLeftRightFrontSide(), 0);
        Assert.assertEquals(expectedWheelDistLeftRightBackSide, physicalVehicle.getWheelDistLeftRightBackSide(), 0);
        Assert.assertEquals(expectedWheelDistToFront, physicalVehicle.getWheelDistToFront(), 0);
        Assert.assertEquals(expectedWheelDistToBack, physicalVehicle.getWheelDistToBack(), 0);

        // Test internal values
        Assert.assertEquals(PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR, physicalVehicle.getPhysicalObjectType());
        Assert.assertFalse(physicalVehicle.getError());
        Assert.assertFalse(physicalVehicle.getCollision());
        Assert.assertTrue(physicalVehicle.getPhysicalVehicleInitialised());

        // Test physical values
        Assert.assertTrue(MathHelper.vectorEquals(expectedForce, physicalVehicle.getForce(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedGeometryPosition, physicalVehicle.getGeometryPosition(), 0.00000001));
        Assert.assertEquals(expectedWheelRotationRate, physicalVehicle.getVDM().getValue("omega_wheel_1"), 0);
        Assert.assertEquals(expectedWheelRotationRate, physicalVehicle.getVDM().getValue("omega_wheel_2"), 0);
        Assert.assertEquals(expectedWheelRotationRate, physicalVehicle.getVDM().getValue("omega_wheel_3"), 0);
        Assert.assertEquals(expectedWheelRotationRate, physicalVehicle.getVDM().getValue("omega_wheel_4"), 0);
    }
    
    private Vehicle createStandardVehicle(PhysicalVehicleBuilder physicalVehicleBuilder) {
    	EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
		EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
		InstantBus bus = new InstantBus(eeSimulator);
		eeVehicleBuilder.createAllSensorsNActuators(bus);
		return new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
    }
}
