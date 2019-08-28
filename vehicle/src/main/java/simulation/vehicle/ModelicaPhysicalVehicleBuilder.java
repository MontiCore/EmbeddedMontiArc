/* (c) https://github.com/MontiCore/monticore */
package simulation.vehicle;

import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * Abstract Builder class for a ModelicaPhysicalVehicle to avoid complex constructors
 */
public class ModelicaPhysicalVehicleBuilder extends PhysicalVehicleBuilder {

    /**
     * Constructor
     */
    public ModelicaPhysicalVehicleBuilder(){
        // Class has no uninitialized fields
    }

    /**
     * Function that returns a ModelicaPhysicalVehicle with the attributes currently stored in the builder
     *
     * @return ModelicaPhysicalVehicle that was built with the builder
     */
    @Override
    public PhysicalVehicle buildPhysicalVehicle(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle(true);

        if(this.velocity.isPresent()){
            // Get rotation
            RealMatrix rotation;
            if(this.rotation.isPresent()){
                rotation = ModelicaPhysicalVehicle.coordinateRotation.multiply(new BlockRealMatrix(this.rotation.get().getMatrix()));
            }else{
                rotation = ModelicaPhysicalVehicle.coordinateRotation.copy();
            }
            // Compute velocity in local coordinates
            RealVector localVelocity = rotation.transpose().operate(this.velocity.get());
            // Set initial velocity values
            ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("v_x_0", localVelocity.getEntry(0));
            ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("v_y_0", localVelocity.getEntry(1));
            // Get wheelRadius
            double wheelRadius;
            if(this.wheelRadius.isPresent()){
                wheelRadius = this.wheelRadius.get();
            }else {
                VehicleDynamicsModel model = new VehicleDynamicsModel();
                model.initialise();
                wheelRadius = model.getValue("r_nom");
            }
            // Set initial wheel rotation rates
            ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("omega_wheel_1_0", localVelocity.getEntry(0) / wheelRadius);
            ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("omega_wheel_2_0", localVelocity.getEntry(0) / wheelRadius);
            ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("omega_wheel_3_0", localVelocity.getEntry(0) / wheelRadius);
            ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("omega_wheel_4_0", localVelocity.getEntry(0) / wheelRadius);
        }
        if(this.angularVelocity.isPresent()){
            // Get rotation
            RealMatrix rotation;
            if(this.rotation.isPresent()){
                rotation = ModelicaPhysicalVehicle.coordinateRotation.multiply(new BlockRealMatrix(this.rotation.get().getMatrix()));
            }else{
                rotation = ModelicaPhysicalVehicle.coordinateRotation.copy();
            }
            // Compute angular velocity in local coordinates
            RealVector localAngularVelocity = rotation.transpose().operate(this.angularVelocity.get());
            // Set initial angular velocity values
            ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("omega_z_0", localAngularVelocity.getEntry(2));
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
}
