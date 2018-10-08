package simulation.vehicle;

import com.google.gson.Gson;
import commons.simulation.PhysicalObjectType;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.junit.*;
import simulation.util.Log;
import simulation.util.MathHelper;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Optional;

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
    public void buildDefaultVehicle(){
        // Build default car
        MassPointPhysicalVehicleBuilder builder = new MassPointPhysicalVehicleBuilder();
        MassPointPhysicalVehicle physicalVehicle = (MassPointPhysicalVehicle) builder.buildPhysicalVehicle();

        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        // Calculate expected values
        double height = physicalVehicle.getHeight();
        RealVector expectedPosition = new ArrayRealVector(new double[]{0.0, 0.0, - height/2});
        Rotation expectedRot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, 0.0);
        RealMatrix expectedRotation = new BlockRealMatrix(expectedRot.getMatrix());
        RealVector expectedVelocity = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        RealVector expectedAngularVelocity = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        RealVector expectedForce = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        RealVector expectedGeometryPosition = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});


        // Test not set/default parameters
        Assert.assertEquals(Optional.empty(), vehicle.getControllerBus());
        Assert.assertEquals(Optional.empty(), vehicle.getController());
        Assert.assertEquals(Optional.empty(), vehicle.getNavigation());
        Assert.assertEquals(Vehicle.VEHICLE_DEFAULT_MASS, vehicle.getMass(), 0);

        // Test internal values
        Assert.assertEquals(PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR, physicalVehicle.getPhysicalObjectType());
        Assert.assertFalse(physicalVehicle.getError());
        Assert.assertFalse(physicalVehicle.getCollision());
        Assert.assertTrue(physicalVehicle.getPhysicalVehicleInitialized());

        // Test physical values
        Assert.assertTrue(MathHelper.vectorEquals(expectedPosition, physicalVehicle.getPosition(), 0.00000001));
        Assert.assertTrue(MathHelper.matrixEquals(expectedRotation, physicalVehicle.getRotation(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedVelocity, physicalVehicle.getVelocity(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedAngularVelocity, physicalVehicle.getAngularVelocity(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedForce, physicalVehicle.getForce(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedGeometryPosition, physicalVehicle.getGeometryPosition(), 0.00000001));
        // todo mass distribution and distances
    }

    @Test
    public void buildCustomVehicle(){
        // Build custom car
        MassPointPhysicalVehicleBuilder builder = new MassPointPhysicalVehicleBuilder();
        builder.setMass(1000.0);
        MassPointPhysicalVehicle physicalVehicle = (MassPointPhysicalVehicle) builder.buildPhysicalVehicle();

        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        // Calculate expected values
        double height = physicalVehicle.getHeight();
        RealVector expectedPosition = new ArrayRealVector(new double[]{0.0, 0.0, - height/2});
        Rotation expectedRot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, 0.0);
        RealMatrix expectedRotation = new BlockRealMatrix(expectedRot.getMatrix());
        RealVector expectedVelocity = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        RealVector expectedAngularVelocity = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        RealVector expectedForce = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        RealVector expectedGeometryPosition = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});

        // Test custom set parameters
        Assert.assertEquals(1000.0, vehicle.getMass(), 0);

        // Test internal values
        Assert.assertEquals(PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR, physicalVehicle.getPhysicalObjectType());
        Assert.assertFalse(physicalVehicle.getError());
        Assert.assertFalse(physicalVehicle.getCollision());
        Assert.assertTrue(physicalVehicle.getPhysicalVehicleInitialized());

        // Test physical values
        Assert.assertTrue(MathHelper.vectorEquals(expectedPosition, physicalVehicle.getPosition(), 0.00000001));
        Assert.assertTrue(MathHelper.matrixEquals(expectedRotation, physicalVehicle.getRotation(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedVelocity, physicalVehicle.getVelocity(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedAngularVelocity, physicalVehicle.getAngularVelocity(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedForce, physicalVehicle.getForce(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedGeometryPosition, physicalVehicle.getGeometryPosition(), 0.00000001));
        // todo mass distribution and distances
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
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().loadFromFile(testFile);

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
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();

        checkTheCar(properties, physicalVehicle);
    }

    /**
     * Checks various vehicle properties and compares them with the initially created one, to assure correct loading.
     *
     * @param physicalVehicle a vehicle to check against the one created in the setup
     */
    private void checkTheCar(MassPointPhysicalVehicleBuilder.ParsableVehicleProperties properties, PhysicalVehicle physicalVehicle) {
        Assert.assertTrue(MathHelper.vectorEquals(properties.getPosition(), physicalVehicle.getPosition(), 0.00000001));

        Assert.assertTrue(MathHelper.matrixEquals(properties.getRotation(), physicalVehicle.getRotation(), 0.00000001));

        Assert.assertTrue(MathHelper.vectorEquals(properties.getVelocity(), physicalVehicle.getVelocity(), 0.00000001));

        Assert.assertTrue(MathHelper.vectorEquals(properties.getAngularVelocity(), physicalVehicle.getAngularVelocity(), 0.00000001));

        Assert.assertEquals(properties.getMass(), physicalVehicle.getSimulationVehicle().getMass(), 0);

        Assert.assertEquals(properties.getWidth(), physicalVehicle.getSimulationVehicle().getWidth(), 0);
        Assert.assertEquals(properties.getLength(), physicalVehicle.getSimulationVehicle().getLength(), 0);
        Assert.assertEquals(properties.getHeight(), physicalVehicle.getSimulationVehicle().getHeight(), 0);

        Assert.assertEquals(properties.getWheelRadius(), physicalVehicle.getSimulationVehicle().getWheelRadius(), 0);
        Assert.assertEquals(properties.getWheelDistToFront(), physicalVehicle.getSimulationVehicle().getWheelDistToFront(), 0);
        Assert.assertEquals(properties.getWheelDistToBack(), physicalVehicle.getSimulationVehicle().getWheelDistToBack(), 0);
        Assert.assertEquals(properties.getWheelDistLeftRightFrontSide(), physicalVehicle.getSimulationVehicle().getWheelDistLeftRightFrontSide(), 0);
        Assert.assertEquals(properties.getWheelDistLeftRightBackSide(), physicalVehicle.getSimulationVehicle().getWheelDistLeftRightBackSide(), 0);

        /*VehicleActuator b;
        for(VehicleActuator a : properties.getActuators()){
            b = physicalVehicle.getSimulationVehicle().getVehicleActuator(a.getActuatorType());
            Assert.assertEquals(a.getActuatorValueMin(), b.getActuatorValueMin(), 0);
            Assert.assertEquals(a.getActuatorValueMax(), b.getActuatorValueMax(), 0);
            Assert.assertEquals(a.getActuatorValueChangeRate(), b.getActuatorValueChangeRate(), 0);
            Assert.assertEquals(a.getActuatorValueTarget(), b.getActuatorValueTarget(), 0);
            Assert.assertEquals(a.getActuatorValueCurrent(), b.getActuatorValueCurrent(), 0);
        }*/
    }
}
