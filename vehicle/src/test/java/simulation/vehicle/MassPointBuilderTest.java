package simulation.vehicle;

import com.google.gson.Gson;
import org.junit.*;
import simulation.util.Log;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Created by samuel on 15.12.16.
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

    /**
     * In this setup method a valid JSON car file and String is created, which is later on used.
     * The created file is registered for deletion on VM exit.
     */
    @Before
    public void setUp() {
        PhysicalVehicle v = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        carProps = new MassPointPhysicalVehicleBuilder.ParsableVehicleProperties(v);

        testFileAsFile = new File(testFile);
        testFileAsFile.deleteOnExit();
    }

    /**
     * Testing the loading a JSON serialized car from a file and construct the object using the @{@link PhysicalVehicleBuilder}.
     *
     * @throws IOException
     */
    @Test
    public void testLoadPropertiesFromFile() throws IOException {

        Gson g = new Gson();
        String json = g.toJson(carProps, MassPointPhysicalVehicleBuilder.ParsableVehicleProperties.class);

        FileWriter fooWriter = new FileWriter(testFile, false);
        fooWriter.write(json);
        fooWriter.flush();
        fooWriter.close();

        MassPointPhysicalVehicleBuilder b = new MassPointPhysicalVehicleBuilder();

        PhysicalVehicle v = b.loadPropertiesFromFile(new File(testFile));
        checkTheCar(v);
    }

    /**
     * Testing storing a car in a JSON file.
     * This is done by first storing it in a file and then load it again and check if all properties remained the same.
     */
    @Test
    public void testStoreJSONInFile() throws IOException {
        new MassPointPhysicalVehicleBuilder().storeJSONInFile(testFileAsFile);
        PhysicalVehicle v = new MassPointPhysicalVehicleBuilder().loadPropertiesFromFile(testFileAsFile);
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
