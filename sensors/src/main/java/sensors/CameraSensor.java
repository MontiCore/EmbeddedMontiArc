/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package sensors;

import com.jhlabs.image.MotionBlurFilter;
import com.jhlabs.image.PerspectiveFilter;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.PhysicalObject;
import ij.ImagePlus;
import ij.process.ImageProcessor;
import org.apache.commons.math3.linear.ArrayRealVector;
import sensors.abstractsensors.AbstractSensor;
import simulation.environment.World;
import simulation.environment.WorldModel;
import simulation.environment.object.House;
import simulation.environment.visualisationadapter.implementation.EnvironmentContainer2D;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
import simulation.vehicle.PhysicalVehicle;
import simulation.simulator.*;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.BufferedImageOp;
import java.awt.image.ConvolveOp;
import java.awt.image.Kernel;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

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

    public Optional<Image> getOriginalImage() {
        return originalImage;
    }

    public CameraSensor(PhysicalVehicle physicalVehicle) {
        super(physicalVehicle);
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_CAMERA;
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
            // get the current used vehicle
            PhysicalVehicle vehicle = this.getPhysicalVehicle();

            // get objects in the environment
            World world = WorldModel.getInstance();
            Collection<House> houses = ((EnvironmentContainer2D)world.getContainer()).getHouses();
            Collection<EnvNode> trees = world.getContainer().getTrees();
            Collection<EnvStreet> nodes = world.getContainer().getStreets();
            Collection<EnvNode> intersection = new ArrayList<EnvNode>();
            for(EnvStreet street: nodes){
                Collection<EnvNode> intersectionode = street.getIntersections();
                for(EnvNode inter : intersectionode){
                    if(!intersection.contains(inter)){
                        intersection.add(inter);
                    }
                }
            }
            Collection<EnvNode> intersection1 = intersection;
            double range = 100;

            // collection of buildings which are in the detecting range of the camera sensor
            Collection<House> detected_buildings = houses;
            detected_buildings.clear(); // to get an empty collection of buildings

            // collection of trees which are in the detecting range of the camera sensor
            Collection<EnvNode> detected_trees = trees;
            detected_trees.clear(); // to get an empty collection of trees

            // TODO: to get all other vehicles in the simulation

            Simulator simulator = Simulator.getSharedInstance();
            List<PhysicalObject> OtherVehicle = simulator.getPhysicalObjects();
            // detect all buildings in the detecting range of the camera sensor
            for (House house : houses){
                double distance = vehicle.getGeometryPosition().getDistance(house.getGeometryPosition());
                if (distance < range){
                    detected_buildings.add(house);
                }
            }

            // detect all trees in the detecting range of the camera sensor
            for (EnvNode tree : trees){
                double distance = vehicle.getGeometryPosition().getDistance(new ArrayRealVector(new double[]{
                        tree.getX().doubleValue(),
                        tree.getY().doubleValue(),
                        tree.getZ().doubleValue()
                }));
                if (distance < range){
                    detected_trees.add(tree);
                }
            }
            // detect all vehicles in the detecting range of the camera sensor
            double distance_init = 200.0;
            EnvNode nearest_intersection = null;
            for (EnvNode inter : intersection){
                double distance = vehicle.getGeometryPosition().getDistance(new ArrayRealVector(new double[]{
                        inter.getX().doubleValue(),
                        inter.getY().doubleValue(),
                        inter.getZ().doubleValue()
                }));

                if(distance <= distance_init) {
                    nearest_intersection = inter;
                    distance_init = distance;
                }
            }


            Drawer drawer = new Drawer();

            // draw sky & ground & street
            drawer.drawBackground();

            // draw street
            if (nearest_intersection != null){
                double distFrontLeftToInter = vehicle.getFrontLeftWheelGeometryPosition().getDistance(getGeometryPosition(nearest_intersection));
                double distFrontRightToInter = vehicle.getFrontRightWheelGeometryPosition().getDistance(getGeometryPosition(nearest_intersection));

                double distBackLeftToInter = vehicle.getBackLeftWheelGeometryPosition().getDistance(getGeometryPosition(nearest_intersection));
                double distBackRightToInter = vehicle.getBackRightWheelGeometryPosition().getDistance(getGeometryPosition(nearest_intersection));
                double distToInter = vehicle.getGeometryPosition().getDistance(getGeometryPosition(nearest_intersection));
                drawer.drawIntersection(distFrontLeftToInter,distFrontRightToInter,distBackLeftToInter,distBackRightToInter,distToInter);


            }

            // draw buildings
            for (House detected_building : detected_buildings){
                double distFrontLeftToBuilding = vehicle.getFrontLeftWheelGeometryPosition().getDistance(detected_building.getGeometryPosition());
                double distFrontRightToBuilding = vehicle.getFrontRightWheelGeometryPosition().getDistance(detected_building.getGeometryPosition());
                double distBackLeftToBuilding = vehicle.getBackLeftWheelGeometryPosition().getDistance(detected_building.getGeometryPosition());
                double distBackRightToBuilding = vehicle.getBackRightWheelGeometryPosition().getDistance(detected_building.getGeometryPosition());
                double distToBuilding = vehicle.getGeometryPosition().getDistance(detected_building.getGeometryPosition());

                drawer.drawBuilding(distToBuilding, distFrontLeftToBuilding, distFrontRightToBuilding, distBackLeftToBuilding, distBackRightToBuilding);
            }

            // draw trees
            for (EnvNode detected_tree : detected_trees){
                double distFrontLeftToTree = vehicle.getFrontLeftWheelGeometryPosition().getDistance(getGeometryPosition(detected_tree));
                double distFrontRightToTree = vehicle.getFrontRightWheelGeometryPosition().getDistance(getGeometryPosition(detected_tree));

                double distBackLeftToTree = vehicle.getBackLeftWheelGeometryPosition().getDistance(getGeometryPosition(detected_tree));
                double distBackRightToTree = vehicle.getBackRightWheelGeometryPosition().getDistance(getGeometryPosition(detected_tree));
                double distToTree = vehicle.getGeometryPosition().getDistance(getGeometryPosition(detected_tree));

                drawer.drawTree(distToTree, distFrontLeftToTree, distFrontRightToTree, distBackLeftToTree, distBackRightToTree);
            }

            // TODO: draw vehicles
            for (PhysicalObject car : OtherVehicle) {
                if(car instanceof PhysicalVehicle){
                    PhysicalVehicle vehicle1 = (PhysicalVehicle) car;
                    double distFrontLeftToCar = vehicle.getFrontLeftWheelGeometryPosition().getDistance(vehicle1.getGeometryPosition());
                    double distFrontRightToCar = vehicle.getFrontRightWheelGeometryPosition().getDistance(vehicle1.getGeometryPosition());

                    double distBackLeftToCar = vehicle.getBackLeftWheelGeometryPosition().getDistance(vehicle1.getGeometryPosition());
                    double distBackRightToCar = vehicle.getBackRightWheelGeometryPosition().getDistance(vehicle1.getGeometryPosition());
                    double distToCar = vehicle.getGeometryPosition().getDistance(vehicle1.getGeometryPosition());
                    drawer.drawVehicle( distToCar,  distFrontLeftToCar, distBackLeftToCar);
                }
            }



            this.value = drawer.getImage();
            this.bi = drawer.getBImage();

        } catch (Exception e) {
            fail();
        }


    }

    private ArrayRealVector getGeometryPosition(EnvNode obj) {
        return new ArrayRealVector(new double[]{
                obj.getX().doubleValue(),
                obj.getY().doubleValue(),
                obj.getZ().doubleValue()
        });
    }

    private void fail() {
    }



    private BufferedImage cropImage(ImagePlus imp, int x, int y, int width, int height){

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
        try{
            bi_blur= bi;
        }
        catch( Exception e ){
            System.out.print( e.toString() );
            return bi;
        }
        BufferedImage bi_blurred = new BufferedImage( bi_blur.getWidth() , bi_blur.getHeight() , bi_blur.getType() );
        float[] matrix = {
                1f/273,4f/273,7f/273,4f/273,1f/273,
                4f/273,16f/273,26f/273,16f/273,4f/273,
                7f/273,26f/273,41f/273,26f/273,7f/273,
                4f/273,16f/273,26f/273,16f/273,4f/273,
                1f/273,4f/273,7f/273,4f/273,1f/273
        };
        BufferedImageOp op = new ConvolveOp( new Kernel( 5, 5, matrix ) );
        bi_blurred = op.filter( bi_blur, bi_blurred );

        return bi_blurred;
    }

    // add additive white gaussian noise
    public static BufferedImage awgn(BufferedImage bi) {

        double mean = 0;
        double sigma = 30;
        BufferedImage originalImage = bi;

        double variance = sigma*sigma;

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

                double variable_x = Math.sqrt(-2*Math.log(a)) * Math.cos(2*Math.PI*b);
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