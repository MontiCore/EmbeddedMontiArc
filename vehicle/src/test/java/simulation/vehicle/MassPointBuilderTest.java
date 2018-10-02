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
import java.io.FileWriter;
import java.io.IOException;
import java.util.Optional;

/**
 * Class that tests the MassPointPhysicalVehicleBuilder class
 */
public class MassPointBuilderTest {
    private MassPointPhysicalVehicleBuilder.ParsableVehicleProperties carProps;
    private String testFile = "car_test.json";
    private File testFileAsFile;

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
    public void testLoadPropertiesFromFile() throws IOException {
        PhysicalVehicle v = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        carProps = new MassPointPhysicalVehicleBuilder.ParsableVehicleProperties(v);

        testFileAsFile = new File(testFile);
        testFileAsFile.deleteOnExit();

        Gson g = new Gson();
        String json = g.toJson(carProps, MassPointPhysicalVehicleBuilder.ParsableVehicleProperties.class);

        FileWriter fooWriter = new FileWriter(testFile, false);
        fooWriter.write(json);
        fooWriter.flush();
        fooWriter.close();

        MassPointPhysicalVehicleBuilder b = new MassPointPhysicalVehicleBuilder();

        v = b.loadPropertiesFromFile(new File(testFile));
        checkTheCar(v);
    }

    /**
     * Testing storing a car in a JSON file.
     * This is done by first storing it in a file and then load it again and check if all properties remained the same.
     */
    @Test
    public void testStoreJSONInFile() throws IOException {
        PhysicalVehicle v = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        carProps = new MassPointPhysicalVehicleBuilder.ParsableVehicleProperties(v);

        testFileAsFile = new File(testFile);
        testFileAsFile.deleteOnExit();

        new MassPointPhysicalVehicleBuilder().storeJSONInFile(testFileAsFile);
        v = new MassPointPhysicalVehicleBuilder().loadPropertiesFromFile(testFileAsFile);
        checkTheCar(v);
    }

    /**
     * Checks various vehicle properties and compares them with the initially created one, to assure correct loading.
     *
     * @param v a vehicle to check against the one created in the setup
     */
    private void checkTheCar(PhysicalVehicle v) {

        Assert.assertEquals(carProps.getHeight(), v.getSimulationVehicle().getHeight(), 0);
        Assert.assertEquals(carProps.getWidth(), v.getSimulationVehicle().getWidth(), 0);
        Assert.assertEquals(carProps.getLength(), v.getSimulationVehicle().getLength(), 0);
        Assert.assertEquals(carProps.getApproxMaxTotalVelocity(), v.getSimulationVehicle().getApproxMaxTotalVelocity(), 0);
        Assert.assertEquals(carProps.getMass(), v.getSimulationVehicle().getMass(), 0);
        Assert.assertEquals(carProps.getWheelRadius(), v.getSimulationVehicle().getWheelRadius(), 0);
        Assert.assertEquals(carProps.getWheelDistToFront(), v.getSimulationVehicle().getWheelDistToFront(), 0);
        Assert.assertEquals(carProps.getWheelDistToBack(), v.getSimulationVehicle().getWheelDistToBack(), 0);
        Assert.assertEquals(carProps.getWheelDistLeftRightFrontSide(), v.getSimulationVehicle().getWheelDistLeftRightFrontSide(), 0);
        Assert.assertEquals(carProps.getWheelDistLeftRightBackSide(), v.getSimulationVehicle().getWheelDistLeftRightBackSide(), 0);

        Assert.assertSame(carProps.getType(), v.getPhysicalObjectType());

    }
}
