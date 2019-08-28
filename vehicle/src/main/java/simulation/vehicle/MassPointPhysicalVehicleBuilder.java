/* (c) https://github.com/MontiCore/monticore */
package simulation.vehicle;

import com.google.gson.Gson;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;

/**
 * Abstract Builder class for a MassPointPhysicalVehicle to avoid complex constructors
 */
public class MassPointPhysicalVehicleBuilder extends PhysicalVehicleBuilder {

    /**
     * Constructor
     */
    public MassPointPhysicalVehicleBuilder(){
        // Class has no uninitialized fields
    }

    /**
     * Function that returns a MassPointPhysicalVehicle with the attributes currently stored in the builder
     *
     * @return MassPointPhysicalVehicle that was built with the builder
     */
    @Override
    public PhysicalVehicle buildPhysicalVehicle(){
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle(true);

        if(this.velocity.isPresent()){
            // Get rotation
            RealMatrix rotation;
            if(this.rotation.isPresent()){
                rotation = new BlockRealMatrix(this.rotation.get().getMatrix());
            }else{
                rotation = new BlockRealMatrix(new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, 0.0).getMatrix());
            }
            // Compute velocity in local coordinates
            RealVector localVelocity = rotation.transpose().operate(this.velocity.get());
            // Set velocity
            physicalVehicle.setVelocity(localVelocity);
        }
        if(this.angularVelocity.isPresent()){
            // Get rotation
            RealMatrix rotation;
            if(this.rotation.isPresent()){
                rotation = new BlockRealMatrix(this.rotation.get().getMatrix());
            }else{
                rotation = new BlockRealMatrix(new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, 0.0).getMatrix());
            }
            // Compute angular velocity in local coordinates
            RealVector localAngularVelocity = rotation.transpose().operate(this.angularVelocity.get());
            // Set angular velocity
            physicalVehicle.setAngularVelocity(localAngularVelocity);
        }

        this.mass.ifPresent(physicalVehicle::setMass);

        this.width.ifPresent(physicalVehicle::setWidth);
        this.length.ifPresent(physicalVehicle::setLength);
        this.height.ifPresent(physicalVehicle::setHeight);

        this.wheelRadius.ifPresent(physicalVehicle.getSimulationVehicle()::setWheelRadius);
        this.wheelDistLeftRightFrontSide.ifPresent(physicalVehicle.getSimulationVehicle()::setWheelDistLeftRightFrontSide);
        this.wheelDistLeftRightBackSide.ifPresent(physicalVehicle.getSimulationVehicle()::setWheelDistLeftRightBackSide);
        this.wheelDistToFront.ifPresent(physicalVehicle.getSimulationVehicle()::setWheelDistToFront);
        this.wheelDistToBack.ifPresent(physicalVehicle.getSimulationVehicle()::setWheelDistToBack);

        this.controllerBus.ifPresent(physicalVehicle.getSimulationVehicle()::setControllerBus);
        this.controller.ifPresent(physicalVehicle.getSimulationVehicle()::setController);
        this.navigation.ifPresent(physicalVehicle.getSimulationVehicle()::setNavigation);

        physicalVehicle.initPhysics();

        this.position.ifPresent(physicalVehicle::setPosition);
        this.rotation.ifPresent(rotation -> physicalVehicle.setRotation(new BlockRealMatrix(rotation.getMatrix())));
        //TODO: Add all default sensor per default to the physical vehicle using the sensor util
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
    public MassPointPhysicalVehicle loadFromFile(File file) throws IOException {
        String jsonContents = new String(Files.readAllBytes(file.toPath()));
        Gson g = new Gson();
        ParsableVehicleProperties data = g.fromJson(jsonContents, ParsableVehicleProperties.class);

        MassPointPhysicalVehicleBuilder builder = new MassPointPhysicalVehicleBuilder();

        builder.setPosition(data.getPosition());
        builder.setRotation(data.getRotation());
        builder.setVelocity(data.getVelocity());
        builder.setAngularVelocity(data.getAngularVelocity());

        builder.setMass(data.getMass());

        builder.setWidth(data.getWidth());
        builder.setLength(data.getLength());
        builder.setHeight(data.getHeight());

        builder.setWheelRadius(data.getWheelRadius());
        builder.setWheelDistLeftRightFrontSide(data.getWheelDistLeftRightFrontSide());
        builder.setWheelDistLeftRightBackSide(data.getWheelDistLeftRightBackSide());
        builder.setWheelDistToFront(data.getWheelDistToFront());
        builder.setWheelDistToBack(data.getWheelDistToBack());

        return (MassPointPhysicalVehicle) builder.buildPhysicalVehicle();
    }

    /**
     * Build a car with the currently stored attributes and stores JSON serialized in a File on the mass storage.
     * Be careful, default behavior is to overwrite existing files.
     *
     * @param whereToStore file to store the JSON
     * @return current instance of the Builder
     * @throws IOException thrown if the given path cannot be accessed.
     */
    public void storeInFile(File whereToStore) throws IOException {
        PhysicalVehicle physicalVehicle = this.buildPhysicalVehicle();
        ParsableVehicleProperties properties = new MassPointPhysicalVehicleBuilder.ParsableVehicleProperties((MassPointPhysicalVehicle) physicalVehicle);

        Gson g = new Gson();
        String json = g.toJson(properties, MassPointPhysicalVehicleBuilder.ParsableVehicleProperties.class);

        FileWriter fileWriter = new FileWriter(whereToStore, false);

        fileWriter.write(json);
        fileWriter.flush();
        fileWriter.close();
    }

    /**
     * Encapsulation class for all file-parsable data concerning a car.
     * Has to be used as a helper object, as plain serialization methods either fail
     * or are not human readable.
     * The values are just a one to one "copy" of the properties configurable in the builder
     * to be able to reuse the building process as is.
     */
    public static class ParsableVehicleProperties {
        private double positionX;
        private double positionY;
        private double positionZ;

        // Rotation order XYZ, Rotation convention vector operator
        private double rotationX;
        private double rotationY;
        private double rotationZ;

        private double velocityX;
        private double velocityY;
        private double velocityZ;

        private double angularVelocityX;
        private double angularVelocityY;
        private double angularVelocityZ;

        private double mass;

        private double width;
        private double height;
        private double length;

        private double wheelRadius;
        private double wheelDistLeftRightFrontSide;
        private double wheelDistLeftRightBackSide;
        private double wheelDistToFront;
        private double wheelDistToBack;

        //private List<VehicleActuator> actuators;

        public ParsableVehicleProperties(MassPointPhysicalVehicle v) {
            positionX = v.getPosition().getEntry(0);
            positionY = v.getPosition().getEntry(1);
            positionZ = v.getPosition().getEntry(2);

            Rotation rot = new Rotation(v.getRotation().getData(), 0.00000001);
            double[] angles = rot.getAngles(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR);
            rotationX = angles[0];
            rotationY = angles[1];
            rotationZ = angles[2];

            velocityX = v.getVelocity().getEntry(0);
            velocityY = v.getVelocity().getEntry(1);
            velocityZ = v.getVelocity().getEntry(2);

            angularVelocityX = v.getAngularVelocity().getEntry(0);
            angularVelocityY = v.getAngularVelocity().getEntry(1);
            angularVelocityZ = v.getAngularVelocity().getEntry(2);

            mass = v.getSimulationVehicle().getMass();

            width = v.getSimulationVehicle().getWidth();
            height = v.getSimulationVehicle().getHeight();
            length = v.getSimulationVehicle().getLength();

            wheelRadius = v.getSimulationVehicle().getWheelRadius();
            wheelDistLeftRightFrontSide = v.getSimulationVehicle().getWheelDistLeftRightFrontSide();
            wheelDistLeftRightBackSide = v.getSimulationVehicle().getWheelDistLeftRightBackSide();
            wheelDistToFront = v.getSimulationVehicle().getWheelDistToFront();
            wheelDistToBack = v.getSimulationVehicle().getWheelDistToBack();
        }

        public RealVector getPosition(){
            return new ArrayRealVector(new double[]{positionX, positionY, positionZ});
        }

        public Rotation getRotation() {
            return new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, rotationX, rotationY, rotationZ);
        }

        public RealVector getVelocity(){
            return new ArrayRealVector(new double[]{velocityX, velocityY, velocityZ});
        }

        public RealVector getAngularVelocity(){
            return new ArrayRealVector(new double[]{angularVelocityX, angularVelocityY, angularVelocityZ});
        }

        public double getMass() {
            return mass;
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
    }
}
