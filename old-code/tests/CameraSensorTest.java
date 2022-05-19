/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors;

import org.junit.Test;
import de.rwth.montisim.simulation.environment.WorldModel;
import de.rwth.montisim.simulation.environment.weather.WeatherSettings;


import java.awt.*;
import java.awt.image.BufferedImage;

import static org.junit.Assert.fail;


public class CameraSensorTest {
    private Image image;

//    @Before
//    public void LoadImage() {
//        try {
//            InputStream in = getClass().getResourceAsStream("/testImage.jpg");
//            BufferedImage bufferedImage = ImageIO.read(in);
//
//            MassPointPhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
//            PhysicalVehicle physicalVehicle = physicalVehicleBuilder.buildPhysicalVehicle();
//            Optional<Image> optional = Optional.of(bufferedImage);
//            physicalVehicle.getSimulationVehicle().setCameraImage(optional);
//
//            CameraSensor cameraSensor = new CameraSensor(physicalVehicle);
//            cameraSensor.update();
//            cameraSensor.getValue();
//            this.image = cameraSensor.getLeftImage().get();
//            // To see the image
///*            new ImagePlus("Original", cameraSensor.getOriginalImage().get()).show();
//            new ImagePlus("LeftImage", (Image) cameraSensor.getLeftImage().get()).show();
//            new ImagePlus("RightImage", cameraSensor.getRightImage().get()).show();*/
//
//        } catch (Exception e) {
//            fail();
//        }
//
//    }

    @Test
    public void applyPerspectiveFilter() {
        try {
//            BufferedImage filteredImage = CameraSensor
//                    .perspectivefilture((new ImagePlus("Original", this.image).getProcessor()).getBufferedImage(), "right");
//            // ImagePlus ip = new ImagePlus("Filtured", filteredImage);
//            // ip.show();

        } catch (Exception e) {
            fail();
        }

    }

    @Test
    public void applyMotionBlurFilter() {
        try {

//            BufferedImage filteredImage = CameraSensor
//                    .motionBlurFilter((new ImagePlus("Original", this.image).getProcessor()).getBufferedImage());
//
//            // new ImagePlus("MotionBlurFilter", filteredImage).show();
        } catch (Exception e) {
            fail();
        }
    }

    @Test
    public void testBuildings() throws Exception {
     /*   Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(1000);

        // Create physical objects
        PhysicalVehicle notNotified = setupDefaultVehicle();
        PhysicalVehicle firstSecondNotified = setupDefaultVehicle();
        PhysicalVehicle lastSecondNotified =setupDefaultVehicle();
        PhysicalVehicle alwaysNotified =setupDefaultVehicle();

        // Register first, second and fourth physical object
        sim.registerAndPutObject(notNotified, 200.0, 0.0, 0.0); // self car
        sim.registerAndPutObject(firstSecondNotified, 280, 20.0, 0.0);
        sim.registerAndPutObject(alwaysNotified, 0.0, 30.0, 0.0);
        // --------------------

        WorldModel.init("/Aachen_small.osm", new WeatherSettings());
        PhysicalVehicle vehicle = (PhysicalVehicle)sim.getPhysicalObjects().get(0);
        if (vehicle instanceof PhysicalVehicle) {
            CameraSensor cs = new CameraSensor(vehicle);

            // get picture
            cs.calculateValue();

            // process picture
            BufferedImage im = cs.getBiValue();

            //ImageIO.write(im, "JPEG", new FileOutputStream("/Users/christapujun/Desktop/SPP/debug_scene.jpeg"));
            BufferedImage filteredImage = im;
            BufferedImage awgnImage = im;
            try {
                filteredImage = CameraSensor.gaussianBlurFilter(im);
                awgnImage = CameraSensor.awgn(im);

            } catch (Exception e) {
                fail();
            }
            // output image
            //ImageIO.write(im, "JPEG", new FileOutputStream("/Users/christapujun/Desktop/SPP/origin_scene.jpeg"));
            //ImageIO.write(filteredImage, "JPEG", new FileOutputStream("/Users/christapujun/Desktop/SPP/blurred_scene.jpeg"));
            //ImageIO.write(awgnImage, "JPEG", new FileOutputStream("/Users/christapujun/Desktop/SPP/awgn_scene.jpeg"));
          }
          */
    }
}
