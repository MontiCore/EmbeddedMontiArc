package simulation.vehicle;

import com.google.gson.Gson;
import commons.simulation.PhysicalObjectType;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Optional;

/**
 * Abstract Builder class for a MassPointPhysicalVehicle to avoid complex constructors
 */
public class MassPointPhysicalVehicleBuilder extends PhysicalVehicleBuilder {

    /**
     * Constructor
     */
    public MassPointPhysicalVehicleBuilder(){

    }

    /**
     * Function that returns a MassPointPhysicalVehicle with the attributes currently stored in the builder
     *
     * @return MassPointPhysicalVehicle that was built with the builder
     */
    public PhysicalVehicle buildPhysicalVehicle(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();

        physicalVehicle.getSimulationVehicle().setControllerBus(controllerBus);
        physicalVehicle.getSimulationVehicle().setController(controller);
        physicalVehicle.getSimulationVehicle().setNavigation(navigation);

        physicalVehicle.initPhysics();

        return physicalVehicle;
    }

    /**
     * Method takes a file that has to contain a valid JSON representation of a car.
     * It returns a MassPointPhysicalVehicle according to the JSON contents
     *
     * @param file a file containing a valid JSON config for a car
     * @return MassPointPhysicalVehicle according to the JSON contents
     * @throws IOException thrown if the given file could either not be found or accessed/read.
     */
    public MassPointPhysicalVehicle loadPropertiesFromFile(File file) throws IOException {
        String jsonContents = new String(Files.readAllBytes(file.toPath()));
        ParsableVehicleProperties data = new Gson().fromJson(jsonContents, ParsableVehicleProperties.class);

        MassPointPhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();

        for (VehicleActuator a : data.actuators) {
            physicalVehicle.getSimulationVehicle().setActuatorProperties(a.getActuatorType(), a.getActuatorValueMin(), a.getActuatorValueMax(), a.getActuatorValueChangeRate());
        }

        physicalVehicle.getSimulationVehicle().setWidth(data.width);
        physicalVehicle.getSimulationVehicle().setLength(data.length);
        physicalVehicle.getSimulationVehicle().setHeight(data.height);

        physicalVehicle.getSimulationVehicle().setMass(data.mass);
        physicalVehicle.getSimulationVehicle().setWheelRadius(data.wheelRadius);
        physicalVehicle.getSimulationVehicle().setWheelDistLeftRightFrontSide(data.wheelDistLeftRightFrontSide);
        physicalVehicle.getSimulationVehicle().setWheelDistLeftRightBackSide(data.wheelDistLeftRightBackSide);
        physicalVehicle.getSimulationVehicle().setWheelDistToFront(data.wheelDistToFront);
        physicalVehicle.getSimulationVehicle().setWheelDistToBack(data.wheelDistToBack);

        physicalVehicle.getSimulationVehicle().setControllerBus(Optional.empty());
        physicalVehicle.getSimulationVehicle().setController(Optional.empty());
        physicalVehicle.getSimulationVehicle().setNavigation(Optional.empty());

        physicalVehicle.initPhysics();

        physicalVehicle.setPosition(new ArrayRealVector(new double[]{data.posX, data.posY, data.posZ}));

        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, data.rotX, data.rotY, data.rotZ);
        RealMatrix rotation = new BlockRealMatrix(rot.getMatrix());
        physicalVehicle.setRotation(rotation);

        return physicalVehicle;
    }

    /**
     * Build a car with the currently stored attributes and stores JSON serialized in a File on the mass storage.
     * Be careful, default behavior is to overwrite existing files.
     *
     * @param whereToStore file to store the JSON
     * @return current instance of the Builder
     * @throws IOException thrown if the given path cannot be accessed.
     */
    public MassPointPhysicalVehicleBuilder storeJSONInFile(File whereToStore) throws IOException {
        PhysicalVehicle v = this.buildPhysicalVehicle();

        Gson g = new Gson();
        ParsableVehicleProperties carProps = new MassPointPhysicalVehicleBuilder.ParsableVehicleProperties(v);
        String json = g.toJson(carProps, MassPointPhysicalVehicleBuilder.ParsableVehicleProperties.class);

        FileWriter fooWriter = new FileWriter(whereToStore, false);

        fooWriter.write(json);
        fooWriter.flush();
        fooWriter.close();

        return this;
    }

    /**
     * Encapsulation class for all file-parsable data concerning a car.
     * Has to be used as a helper object, as plain serialization methods either fail
     * or are not human readable.
     * The values are just a one to one "copy" of the properties configurable in the builder
     * to be able to reuse the building process as is.
     */
    public static class ParsableVehicleProperties {

        private double width;
        private double height;
        private double length;

        private double approxMaxTotalVelocity;

        private double posX;
        private double posY;
        private double posZ;

        private double rotX;
        private double rotY;
        private double rotZ;

        private double mass;
        private double wheelRadius;
        private double wheelDistLeftRightFrontSide;
        private double wheelDistLeftRightBackSide;
        private double wheelDistToFront;
        private double wheelDistToBack;

        private ArrayList<VehicleActuator> actuators = new ArrayList<>();


        private PhysicalObjectType type;

        public ParsableVehicleProperties(PhysicalVehicle v) {

            width = v.getSimulationVehicle().getWidth();
            height = v.getSimulationVehicle().getHeight();
            length = v.getSimulationVehicle().getLength();

            approxMaxTotalVelocity = v.getSimulationVehicle().getApproxMaxTotalVelocity();

            posX = v.getPosition().getEntry(0);
            posY = v.getPosition().getEntry(1);
            posZ = v.getPosition().getEntry(2);

            mass = v.getSimulationVehicle().getMass();
            wheelRadius = v.getSimulationVehicle().getWheelRadius();

            wheelDistLeftRightFrontSide = v.getSimulationVehicle().getWheelDistLeftRightFrontSide();
            wheelDistLeftRightBackSide = v.getSimulationVehicle().getWheelDistLeftRightBackSide();

            wheelDistToFront = v.getSimulationVehicle().getWheelDistToFront();
            wheelDistToBack = v.getSimulationVehicle().getWheelDistToBack();

            type = v.getPhysicalObjectType();
        }

        public double getWidth() {
            return width;
        }

        public double getHeight() {
            return height;
        }

        public double getLength() {
            return length;
        }

        public double getApproxMaxTotalVelocity() {
            return approxMaxTotalVelocity;
        }

        public double getPosX() {
            return posX;
        }

        public double getPosY() {
            return posY;
        }

        public double getPosZ() {
            return posZ;
        }

        public double getRotX() {
            return rotX;
        }

        public double getRotY() {
            return rotY;
        }

        public double getRotZ() {
            return rotZ;
        }

        public double getMass() {
            return mass;
        }

        public double getWheelRadius() {
            return wheelRadius;
        }

        public double getWheelDistLeftRightFrontSide() {
            return wheelDistLeftRightFrontSide;
        }

        public double getWheelDistLeftRightBackSide() {
            return wheelDistLeftRightBackSide;
        }

        public double getWheelDistToFront() {
            return wheelDistToFront;
        }

        public double getWheelDistToBack() {
            return wheelDistToBack;
        }

        public ArrayList<VehicleActuator> getActuators() {
            return actuators;
        }

        public PhysicalObjectType getType() {
            return type;
        }

        public void setType(PhysicalObjectType type) {
            this.type = type;
        }

    }
}
