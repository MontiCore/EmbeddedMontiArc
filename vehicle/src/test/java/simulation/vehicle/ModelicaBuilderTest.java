package simulation.vehicle;

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
        // Build default car
        ModelicaPhysicalVehicleBuilder builder = new ModelicaPhysicalVehicleBuilder();
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) builder.buildPhysicalVehicle();

        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        // Calculate expected values
        double z = physicalVehicle.getVDM().getValue("z");
        double height = physicalVehicle.getHeight();
        RealVector expectedPosition = new ArrayRealVector(new Double[]{0.0, 0.0, - height/2 + z});
        Rotation expectedRot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, Math.PI / 2);
        RealMatrix expectedRotation = new BlockRealMatrix(expectedRot.getMatrix());
        RealVector expectedVelocity = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        RealVector expectedForce = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        RealVector expectedGeometryPosition = new ArrayRealVector(new Double[]{0.0, 0.0, 0.0});

        // Test not set/default parameters
        Assert.assertEquals(Optional.empty(), vehicle.getControllerBus());
        Assert.assertEquals(Optional.empty(), vehicle.getController());
        Assert.assertEquals(Optional.empty(), vehicle.getNavigation());
        Assert.assertEquals(1750.0, vehicle.getMass(), 0);

        // Test internal values
        Assert.assertEquals(PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR, physicalVehicle.getPhysicalObjectType());
        Assert.assertFalse(physicalVehicle.getError());
        Assert.assertFalse(physicalVehicle.getCollision());
        Assert.assertTrue(physicalVehicle.getPhysicalVehicleInitialized());

        // Test physical values
        Assert.assertTrue(MathHelper.vectorEquals(expectedPosition, physicalVehicle.getPosition(), 0.00000001));
        Assert.assertTrue(MathHelper.matrixEquals(expectedRotation, physicalVehicle.getRotation(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedVelocity, physicalVehicle.getVelocity(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedForce, physicalVehicle.getForce(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedGeometryPosition, physicalVehicle.getGeometryPosition(), 0.00000001));
    }

    @Test
    public void buildCustomVehicle(){
        // Build custom car
        ModelicaPhysicalVehicleBuilder builder = new ModelicaPhysicalVehicleBuilder();
        builder.setMass(1000.0);
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) builder.buildPhysicalVehicle();

        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        // Calculate expected values
        double z = physicalVehicle.getVDM().getValue("z");
        double height = physicalVehicle.getHeight();
        RealVector expectedPosition = new ArrayRealVector(new double[]{0.0, 0.0, - height/2 + z});
        Rotation expectedRot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, Math.PI / 2);
        RealMatrix expectedRotation = new BlockRealMatrix(expectedRot.getMatrix());
        RealVector expectedVelocity = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
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
        Assert.assertTrue(MathHelper.vectorEquals(expectedForce, physicalVehicle.getForce(), 0.00000001));
        Assert.assertTrue(MathHelper.vectorEquals(expectedGeometryPosition, physicalVehicle.getGeometryPosition(), 0.00000001));
    }
}
