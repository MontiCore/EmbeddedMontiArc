/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import com.google.gson.Gson;
import de.rwth.montisim.commons.controller.interfaces.Bus;
import de.rwth.montisim.commons.controller.interfaces.FunctionBlockInterface;
import de.rwth.montisim.commons.simulation.PhysicalObjectType;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import de.rwth.montisim.commons.utils.Vec3;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import de.rwth.montisim.commons.utils.Vec3;
import org.junit.*;

import simulation.EESimulator.EESimulator;
import simulation.bus.InstantBus;
import de.rwth.montisim.simulation.util.Log;
import de.rwth.montisim.simulation.util.MathHelper;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.time.Instant;
import java.util.Optional;

import static org.junit.Assert.*;

/**
 * Class that tests the MassPointPhysicalVehicleBuilder class
 */
public class MassPointBuilderTest {
    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    @Test
    public void buildDefaultVehicle() {
        // Calculate expected values
        double height = PhysicalVehicle.VEHICLE_DEFAULT_HEIGHT;
        Vec3 expectedPosition = new Vec3(new double[]{0.0, 0.0, -height / 2});
        Rotation expectedRot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, 0.0);
        RealMatrix expectedRotation = new BlockRealMatrix(expectedRot.getMatrix());
        Vec3 expectedVelocity = new Vec3(new double[]{0.0, 0.0, 0.0});
        Vec3 expectedAngularVelocity = new Vec3(new double[]{0.0, 0.0, 0.0});
        double expectedMass = PhysicalVehicle.VEHICLE_DEFAULT_MASS;
        double expectedWidth = PhysicalVehicle.VEHICLE_DEFAULT_WIDTH;
        double expectedLength = PhysicalVehicle.VEHICLE_DEFAULT_LENGTH;
        double expectedHeight = PhysicalVehicle.VEHICLE_DEFAULT_HEIGHT;
        double expectedWheelRadius = PhysicalVehicle.VEHICLE_DEFAULT_WHEEL_RADIUS;
        double expectedWheelDistLeftRightFrontSide = PhysicalVehicle.VEHICLE_DEFAULT_WHEEL_TRACK_WIDTH_FRONT;
        double expectedWheelDistLeftRightBackSide = PhysicalVehicle.VEHICLE_DEFAULT_WHEEL_TRACK_WIDTH_REAR;
        double expectedWheelDistToFront = PhysicalVehicle.VEHICLE_DEFAULT_WHEEL_DIST_TO_FRONT;
        double expectedWheelDistToBack = PhysicalVehicle.VEHICLE_DEFAULT_WHEEL_DIST_TO_BACK;

        // Calculate expected remaining values
        Vec3 expectedForce = new Vec3(new double[]{0.0, 0.0, 0.0});
        Vec3 expectedTorque = new Vec3(3);
        Vec3 expectedGeometryPosition = new Vec3(new double[]{0.0, 0.0, 0.0});

        // Build default car
        PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
        MassPointPhysicalVehicle physicalVehicle = (MassPointPhysicalVehicle) vehicle.getPhysicalVehicle();

        // Test not set/default parameters
        assertTrue(MathHelper.vectorEquals(expectedPosition, physicalVehicle.getPosition(), 0.00000001));
        assertTrue(MathHelper.matrixEquals(expectedRotation, physicalVehicle.getRotation(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(expectedVelocity, physicalVehicle.getVelocity(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(expectedAngularVelocity, physicalVehicle.getAngularVelocity(), 0.00000001));
        assertEquals(expectedMass, physicalVehicle.getMass(), 0);
        assertEquals(expectedWidth, physicalVehicle.getWidth(), 0);
        assertEquals(expectedLength, physicalVehicle.getLength(), 0);
        assertEquals(expectedHeight, physicalVehicle.getHeight(), 0);
        assertEquals(expectedWheelRadius, physicalVehicle.getWheelRadius(), 0);
        assertEquals(expectedWheelDistLeftRightFrontSide, physicalVehicle.getWheelDistLeftRightFrontSide(), 0);
        assertEquals(expectedWheelDistLeftRightBackSide, physicalVehicle.getWheelDistLeftRightBackSide(), 0);
        assertEquals(expectedWheelDistToFront, physicalVehicle.getWheelDistToFront(), 0);
        assertEquals(expectedWheelDistToBack, physicalVehicle.getWheelDistToBack(), 0);

        // Test internal values
        assertEquals(PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR, physicalVehicle.getPhysicalObjectType());
        assertFalse(physicalVehicle.getError());
        assertFalse(physicalVehicle.getCollision());
        assertTrue(physicalVehicle.getPhysicalVehicleInitialised());

        // Test physical values
        assertTrue(MathHelper.vectorEquals(expectedForce, physicalVehicle.getForce(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(expectedTorque, physicalVehicle.getTorque(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(expectedGeometryPosition, physicalVehicle.getGeometryPosition(), 0.00000001));
        //TODO: Check mass distribution and distances and values
    }

    @Test
    public void buildCustomVehicle() {
        // Calculate expected values
        Vec3 expectedPosition = new Vec3(new double[]{12.0, -3.0, 3.5});
        Rotation expectedRot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 7.0, -1.5, Math.PI / 2);
        RealMatrix expectedRotation = new BlockRealMatrix(expectedRot.getMatrix());
        Vec3 expectedVelocity = new Vec3(new double[]{1.0, -5.725, 12.0});
        Vec3 expectedAngularVelocity = new Vec3(new double[]{7.0, -2.5, 11.75});
        double expectedMass = 1000.0;
        double expectedWidth = 2.125;
        double expectedLength = 10.0;
        double expectedHeight = 3.5;
        double expectedWheelRadius = 0.66;
        double expectedWheelDistLeftRightFrontSide = 1.1;
        double expectedWheelDistLeftRightBackSide = 2;
        double expectedWheelDistToFront = 2.75;
        double expectedWheelDistToBack = 4.25;
        String expectedGlobalId = "global-id";

        // Calculate remaining expected values
        Vec3 expectedForce = new Vec3(new double[]{0.0, 0.0, 0.0});
        Vec3 expectedTorque = new Vec3(3);
        Vec3 expectedGeometryPositionOffset = expectedRotation.operate(new Vec3(new double[]{0.0, 0.0, expectedHeight / 2}));
        Vec3 expectedGeometryPosition = expectedPosition.add(expectedGeometryPositionOffset);

        // Build custom car
        MassPointPhysicalVehicleBuilder builder = new MassPointPhysicalVehicleBuilder();
        builder.setPosition(expectedPosition);
        builder.setRotation(expectedRot);
        builder.setVelocity(expectedVelocity);
        builder.setAngularVelocity(expectedAngularVelocity);
        builder.setMass(expectedMass);
        builder.setWidth(expectedWidth);
        builder.setLength(expectedLength);
        builder.setHeight(expectedHeight);
        builder.setWheelRadius(expectedWheelRadius);
        builder.setWheelDistLeftRightFrontSide(expectedWheelDistLeftRightFrontSide);
        builder.setWheelDistLeftRightBackSide(expectedWheelDistLeftRightBackSide);
        builder.setWheelDistToFront(expectedWheelDistToFront);
        builder.setWheelDistToBack(expectedWheelDistToBack);
        builder.setGlobalId(expectedGlobalId);


        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        Vehicle vehicle = new Vehicle(builder, eeVehicleBuilder);
        MassPointPhysicalVehicle physicalVehicle = (MassPointPhysicalVehicle) vehicle.getPhysicalVehicle();

        // Test custom set parameters
        assertTrue(MathHelper.vectorEquals(expectedPosition, physicalVehicle.getPosition(), 0.00000001));
        assertTrue(MathHelper.matrixEquals(expectedRotation, physicalVehicle.getRotation(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(expectedVelocity, physicalVehicle.getVelocity(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(expectedAngularVelocity, physicalVehicle.getAngularVelocity(), 0.00000001));
        assertEquals(expectedMass, physicalVehicle.getMass(), 0);
        assertEquals(expectedWidth, physicalVehicle.getWidth(), 0);
        assertEquals(expectedLength, physicalVehicle.getLength(), 0);
        assertEquals(expectedHeight, physicalVehicle.getHeight(), 0);
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
        Assert.assertTrue(MathHelper.vectorEquals(expectedTorque, physicalVehicle.getTorque(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedGeometryPosition, physicalVehicle.getGeometryPosition(), 0.00000001));
        //TODO: Check mass distribution and distances and values
    }

    /**
     * Testing the loading a JSON serialized car from a file and construct the object using the @{@link PhysicalVehicleBuilder}.
     */
    @Test
    public void loadFromFileTest() throws IOException {
        // Create file
        File testFile = new File("testFile.json");
        testFile.deleteOnExit();

        // Create default car and store in file
        MassPointPhysicalVehicleBuilder builder = new MassPointPhysicalVehicleBuilder();
        builder.storeInFile(testFile);

        // Load content of file
        String fileContent = new String(Files.readAllBytes(testFile.toPath()));
        Gson g = new Gson();
        MassPointPhysicalVehicleBuilder.ParsableVehicleProperties properties = g.fromJson(fileContent, MassPointPhysicalVehicleBuilder.ParsableVehicleProperties.class);

        // Load car from file
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        Vehicle vehicle = new Vehicle(testFile, eeVehicleBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();

        // Check if both are equal
        checkTheCar(properties, physicalVehicle);
    }

    /**
     * Testing storing a car in a JSON file.
     * This is done by first storing it in a file and then load it again and check if all properties remained the same.
     */
    @Test
    public void storeInFileTest() throws IOException {
        // Create file
        File testFile = new File("testFile.json");
        testFile.deleteOnExit();

        // Create default car and store in file
        MassPointPhysicalVehicleBuilder builder = new MassPointPhysicalVehicleBuilder();
        builder.storeInFile(testFile);

        // Load content of file
        String fileContent = new String(Files.readAllBytes(testFile.toPath()));
        Gson g = new Gson();
        MassPointPhysicalVehicleBuilder.ParsableVehicleProperties properties = g.fromJson(fileContent, MassPointPhysicalVehicleBuilder.ParsableVehicleProperties.class);

        // Create default reference car
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = new Vehicle(massPointBuilder, eeVehicleBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();

        checkTheCar(properties, physicalVehicle);
    }

    /**
     * Checks various vehicle properties and compares them with the initially created one, to assure correct loading.
     *
     * @param physicalVehicle a vehicle to check against the one created in the setup
     */
    private void checkTheCar(MassPointPhysicalVehicleBuilder.ParsableVehicleProperties properties, PhysicalVehicle physicalVehicle) {
        Assert.assertTrue(MathHelper.vectorEquals(properties.getPosition(), physicalVehicle.getPosition(), 0.00000001));

        Assert.assertTrue(MathHelper.matrixEquals(new BlockRealMatrix(properties.getRotation().getMatrix()), physicalVehicle.getRotation(), 0.00000001));

        Assert.assertTrue(MathHelper.vectorEquals(properties.getVelocity(), physicalVehicle.getVelocity(), 0.00000001));

        Assert.assertTrue(MathHelper.vectorEquals(properties.getAngularVelocity(), physicalVehicle.getAngularVelocity(), 0.00000001));

        Assert.assertEquals(properties.getMass(), physicalVehicle.getMass(), 0);

        Assert.assertEquals(properties.getWidth(), physicalVehicle.getWidth(), 0);
        Assert.assertEquals(properties.getLength(), physicalVehicle.getLength(), 0);
        Assert.assertEquals(properties.getHeight(), physicalVehicle.getHeight(), 0);

        Assert.assertEquals(properties.getWheelRadius(), physicalVehicle.getWheelRadius(), 0);
        Assert.assertEquals(properties.getWheelDistToFront(), physicalVehicle.getWheelDistToFront(), 0);
        Assert.assertEquals(properties.getWheelDistToBack(), physicalVehicle.getWheelDistToBack(), 0);
        Assert.assertEquals(properties.getWheelDistLeftRightFrontSide(), physicalVehicle.getWheelDistLeftRightFrontSide(), 0);
        Assert.assertEquals(properties.getWheelDistLeftRightBackSide(), physicalVehicle.getWheelDistLeftRightBackSide(), 0);
    }
}
