/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors;

import com.jhlabs.image.MotionBlurFilter;
import com.jhlabs.image.PerspectiveFilter;
import de.rwth.montisim.commons.controller.commons.BusEntry;
import de.rwth.montisim.commons.simulation.IPhysicalVehicle;
import de.rwth.montisim.commons.simulation.PhysicalObject;
import de.rwth.montisim.commons.simulation.PhysicalObjectType;
import ij.ImagePlus;
import ij.process.ImageProcessor;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import de.rwth.montisim.simulation.environment.World;
import de.rwth.montisim.simulation.environment.WorldModel;
import de.rwth.montisim.simulation.environment.object.House;
import de.rwth.montisim.simulation.environment.visualisationadapter.implementation.EnvironmentContainer2D;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.EnvNode;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.EnvStreet;


import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.BufferedImageOp;
import java.awt.image.ConvolveOp;
import java.awt.image.Kernel;
import java.util.*;
import java.util.List;

/**
 * Created by Aklima Zaman on 2/8/2017.
 */
public class CameraSensor extends AbstractSensor {
    private Optional<Image> value = Optional.empty();    //value is leftImage
    private Optional<Image> rightImage = Optional.empty();
    private Optional<Image> originalImage = Optional.empty();
    private BufferedImage bi;

    // camera parameter ToDo: get them from the real used camera
    private double cameraHFOV = 90.4642140657; // [deg]
    private double focalDistance = 0.1369274884123756; // [m]
    private double baseline = 0.02; // [m]

    //private static World world;
    private final List<IPhysicalVehicle> otherVehicles;

    public Optional<Image> getOriginalImage() {
        return originalImage;
    }

    /**
     *
     * @param phyiscalVehicle
     * @param simulator
     * @param subscribedMessages
     * @param targetsByMessageId
     * @param simulationObjects the objects in the simulation
     */
    public CameraSensor(IPhysicalVehicle phyiscalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                        HashMap<BusEntry, List<EEComponent>> targetsByMessageId, List<PhysicalObject> simulationObjects) {
        super(phyiscalVehicle, simulator, subscribedMessages, targetsByMessageId);
        otherVehicles = new ArrayList<IPhysicalVehicle>();
        for (PhysicalObject object : simulationObjects) {
            if (object.getPhysicalObjectType() == PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR) {
                IPhysicalVehicle otherVehicle = (IPhysicalVehicle) object;
                if (otherVehicle.getId() != this.getPhysicalVehicle().getId()) {
                    otherVehicles.add(otherVehicle);
                }
            }
        }
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_CAMERA;
    }

    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_CAMERA;
    }


    @Override
    public int getDataLength() {
        return 50;
    }

    @Override

    public Object getValue() {

        return value;
    }

    public BufferedImage getBiValue() {
        return bi;
    }

    @Override
    public String getTypeName() {
        return Image.class.getTypeName();
    }

    public Optional<Image> getRightImage() {
        return this.rightImage;
    }

    public Optional<Image> getLeftImage() {
        return this.value;
    }

    @Override
    protected void calculateValue() {
        /*
        Optional<Image> temp = getPhysicalVehicle().getSimulationVehicle().getCameraImage();
        if (temp.isPresent()) {
            this.originalImage = Optional.of(temp.get());
            //TODO: Add noise in Image
            ImagePlus imagePlus = new ImagePlus();
            imagePlus.setImage(temp.get());
            int width = imagePlus.getWidth() / 2;
            this.value = Optional.of(cropImage(imagePlus, 0, 0, width, imagePlus.getHeight()));
            this.rightImage = Optional
                    .of(cropImage(imagePlus, imagePlus.getWidth() / 2, 0, width, imagePlus.getHeight()));
        }
        */

        //Simulator sim = Simulator.getSharedInstance();

        // create image depending on other sensors (to be implemented)
        try {
            // get objects in the environment
            World world = WorldModel.getInstance();
            Collection<House> houses = ((EnvironmentContainer2D) world.getContainer()).getHouses();
            Collection<EnvNode> trees = world.getContainer().getTrees();
            Collection<EnvStreet> nodes = world.getContainer().getStreets();
            Collection<EnvNode> intersection = new ArrayList<EnvNode>();
            for (EnvStreet street : nodes) {
                Collection<EnvNode> intersectionode = street.getIntersections();
                for (EnvNode inter : intersectionode) {
                    if (!intersection.contains(inter)) {
                        intersection.add(inter);
                    }
                }
            }
            //Collection<EnvNode> intersection1 = intersection;
            double range = 100;

            // collection of buildings which are in the detecting range of the camera sensor
            Collection<House> detected_buildings = houses;
            detected_buildings.clear(); // to get an empty collection of buildings

            // collection of trees which are in the detecting range of the camera sensor
            Collection<EnvNode> detected_trees = trees;
            detected_trees.clear(); // to get an empty collection of trees

            // TODO: to get all other vehicles in the simulation

            // detect all buildings in the detecting range of the camera sensor
            IPhysicalVehicle vehicle = this.getPhysicalVehicle();
            for (House house : houses) {
                double distance = vehicle.getGeometryPosition().getDistance(house.getGeometryPosition());
                if (distance < range) {
                    detected_buildings.add(house);
                }
            }

            // detect all trees in the detecting range of the camera sensor
            for (EnvNode tree : trees) {
                double distance = vehicle.getGeometryPosition().getDistance(new Vec3(new double[]{
                        tree.getX().doubleValue(),
                        tree.getY().doubleValue(),
                        tree.getZ().doubleValue()
                }));
                if (distance < range) {
                    detected_trees.add(tree);
                }
            }
            // detect all vehicles in the detecting range of the camera sensor
            double distance_init = 200.0;
            EnvNode nearest_intersection = null;
            for (EnvNode inter : intersection) {
                double distance = vehicle.getGeometryPosition().getDistance(new Vec3(new double[]{
                        inter.getX().doubleValue(),
                        inter.getY().doubleValue(),
                        inter.getZ().doubleValue()
                }));

                if (distance <= distance_init) {
                    nearest_intersection = inter;
                    distance_init = distance;
                }
            }


            Drawer drawer = new Drawer();

            // draw sky & ground & street
            drawer.drawBackground();

            // draw street
            if (nearest_intersection != null) {
                double distFrontLeftToInter = vehicle.getFrontLeftWheelGeometryPosition().getDistance(getGeometryPosition(nearest_intersection));
                double distFrontRightToInter = vehicle.getFrontRightWheelGeometryPosition().getDistance(getGeometryPosition(nearest_intersection));

                double distBackLeftToInter = vehicle.getBackLeftWheelGeometryPosition().getDistance(getGeometryPosition(nearest_intersection));
                double distBackRightToInter = vehicle.getBackRightWheelGeometryPosition().getDistance(getGeometryPosition(nearest_intersection));
                double distToInter = vehicle.getGeometryPosition().getDistance(getGeometryPosition(nearest_intersection));
                drawer.drawIntersection(distFrontLeftToInter, distFrontRightToInter, distBackLeftToInter, distBackRightToInter, distToInter);


            }

            // draw buildings
            for (House detected_building : detected_buildings) {
                double distFrontLeftToBuilding = vehicle.getFrontLeftWheelGeometryPosition().getDistance(detected_building.getGeometryPosition());
                double distFrontRightToBuilding = vehicle.getFrontRightWheelGeometryPosition().getDistance(detected_building.getGeometryPosition());
                double distBackLeftToBuilding = vehicle.getBackLeftWheelGeometryPosition().getDistance(detected_building.getGeometryPosition());
                double distBackRightToBuilding = vehicle.getBackRightWheelGeometryPosition().getDistance(detected_building.getGeometryPosition());
                double distToBuilding = vehicle.getGeometryPosition().getDistance(detected_building.getGeometryPosition());

                drawer.drawBuilding(distToBuilding, distFrontLeftToBuilding, distFrontRightToBuilding, distBackLeftToBuilding, distBackRightToBuilding);
            }

            // draw trees
            for (EnvNode detected_tree : detected_trees) {
                double distFrontLeftToTree = vehicle.getFrontLeftWheelGeometryPosition().getDistance(getGeometryPosition(detected_tree));
                double distFrontRightToTree = vehicle.getFrontRightWheelGeometryPosition().getDistance(getGeometryPosition(detected_tree));

                double distBackLeftToTree = vehicle.getBackLeftWheelGeometryPosition().getDistance(getGeometryPosition(detected_tree));
                double distBackRightToTree = vehicle.getBackRightWheelGeometryPosition().getDistance(getGeometryPosition(detected_tree));
                double distToTree = vehicle.getGeometryPosition().getDistance(getGeometryPosition(detected_tree));

                drawer.drawTree(distToTree, distFrontLeftToTree, distFrontRightToTree, distBackLeftToTree, distBackRightToTree);
            }

            // TODO: draw vehicles
            for (IPhysicalVehicle otherVehicle : otherVehicles) {
                double distFrontLeftToCar = vehicle.getFrontLeftWheelGeometryPosition().getDistance(otherVehicle.getGeometryPosition());
                //double distFrontRightToCar = vehicle.getFrontRightWheelGeometryPosition().getDistance(otherVehicle.getGeometryPosition());

                double distBackLeftToCar = vehicle.getBackLeftWheelGeometryPosition().getDistance(otherVehicle.getGeometryPosition());
                //double distBackRightToCar = vehicle.getBackRightWheelGeometryPosition().getDistance(otherVehicle.getGeometryPosition());
                double distToCar = vehicle.getGeometryPosition().getDistance(otherVehicle.getGeometryPosition());
                drawer.drawVehicle(distToCar, distFrontLeftToCar, distBackLeftToCar);
            }


            this.value = drawer.getImage();
            this.bi = drawer.getBImage();

        } catch (Exception e) {
            fail();
        }


    }

    private Vec3 getGeometryPosition(EnvNode obj) {
        return new Vec3(new double[]{
                obj.getX().doubleValue(),
                obj.getY().doubleValue(),
                obj.getZ().doubleValue()
        });
    }

    private void fail() {
    }


    private BufferedImage cropImage(ImagePlus imp, int x, int y, int width, int height) {

        ImageProcessor ip = imp.getProcessor();
        ip.setInterpolationMethod(ImageProcessor.BILINEAR);

        Rectangle roi = new Rectangle();
        roi.setBounds(x, y, width, height);
        ip.setRoi(roi);
        ImageProcessor cropped = ip.crop();
        return cropped.getBufferedImage();
    }

    /**
     * @return the horizontal field of view (opening angle) in [deg]
     */
    public double getCameraHFOV() {
        return cameraHFOV;
    }

    /**
     * @return the focal distance of the used camera in [m]
     */
    public double getFocalDistance() {
        return focalDistance;
    }

    /**
     * @return the distance between the stereo images in [m]
     */
    public double getBaseline() {
        return baseline;
    }

    public static BufferedImage perspectivefilture(BufferedImage bi, String perspectiveDirection) {
        int w = bi.getWidth();
        int h = bi.getHeight();
        float perspectiveTop = 3f;
        float perspectiveBottom = 3f;

        //TODO: read from artwork profile
        int top3d = (int) (h * perspectiveTop / 10);
        // int top3d = (int) (h * perspectiveTop / 100);
        int bot3d = (int) (h * perspectiveBottom / 100);

        PerspectiveFilter perspectiveFilter = new PerspectiveFilter();
        // Top Left (x/y), Top Right (x/y), Bottom Right (x/y), Bottom Left
        // (x/y)

        if ("right".equalsIgnoreCase(perspectiveDirection)) {
            perspectiveFilter.setCorners(0, 0, w, top3d, w, h - bot3d, 0, h);
        } else {
            perspectiveFilter.setCorners(0, top3d, w, 0, w, h, 0, h - bot3d);
        }
        return perspectiveFilter.filter(bi, null);
    }

    public static BufferedImage motionBlurFilter(BufferedImage bi) {
        MotionBlurFilter blurFilter = new MotionBlurFilter(0.1f, 0.1f, 0.12f, 0.001f);

        return blurFilter.filter(bi, null);
    }

    public static BufferedImage gaussianBlurFilter(BufferedImage bi) {

        BufferedImage bi_blur = null;
        try {
            bi_blur = bi;
        } catch (Exception e) {
            System.out.print(e.toString());
            return bi;
        }
        BufferedImage bi_blurred = new BufferedImage(bi_blur.getWidth(), bi_blur.getHeight(), bi_blur.getType());
        float[] matrix = {
                1f / 273, 4f / 273, 7f / 273, 4f / 273, 1f / 273,
                4f / 273, 16f / 273, 26f / 273, 16f / 273, 4f / 273,
                7f / 273, 26f / 273, 41f / 273, 26f / 273, 7f / 273,
                4f / 273, 16f / 273, 26f / 273, 16f / 273, 4f / 273,
                1f / 273, 4f / 273, 7f / 273, 4f / 273, 1f / 273
        };
        BufferedImageOp op = new ConvolveOp(new Kernel(5, 5, matrix));
        bi_blurred = op.filter(bi_blur, bi_blurred);

        return bi_blurred;
    }

    // add additive white gaussian noise
    public static BufferedImage awgn(BufferedImage bi) {

        double mean = 0;
        double sigma = 30;
        BufferedImage originalImage = bi;

        double variance = sigma * sigma;

        int width = originalImage.getWidth();
        int height = originalImage.getHeight();

        BufferedImage filteredImage = new BufferedImage(width, height, originalImage.getType());

        double a = 0.0;
        double b = 0.0;

        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {

                while (a == 0.0)
                    a = Math.random();
                b = Math.random();

                double variable_x = Math.sqrt(-2 * Math.log(a)) * Math.cos(2 * Math.PI * b);
                double noise = mean + Math.sqrt(variance) * variable_x;

                int gray = new Color(originalImage.getRGB(i, j)).getRed();
                int alpha = new Color(originalImage.getRGB(i, j)).getAlpha();

                double color = gray + noise;
                if (color > 255)
                    color = 255;
                if (color < 0)
                    color = 0;

                int newColor = (int) Math.round(color);

                int newPixel = 0;
                newPixel += alpha;
                newPixel = newPixel << 8;
                newPixel += newColor;
                newPixel = newPixel << 8;
                newPixel += newColor;
                newPixel = newPixel << 8;
                newPixel += newColor;

                filteredImage.setRGB(i, j, newPixel);
            }
        }

        return filteredImage;

    }
}
