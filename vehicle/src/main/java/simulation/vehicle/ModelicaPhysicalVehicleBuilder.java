package simulation.vehicle;

import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * Abstract Builder class for a ModelicaPhysicalVehicle to avoid complex constructors
 */
public class ModelicaPhysicalVehicleBuilder extends PhysicalVehicleBuilder {

    /**
     * Constructor
     */
    public ModelicaPhysicalVehicleBuilder(){

    }

    /**
     * Function that returns a ModelicaPhysicalVehicle with the attributes currently stored in the builder
     *
     * @return ModelicaPhysicalVehicle that was built with the builder
     */
    @Override
    public PhysicalVehicle buildPhysicalVehicle(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();

        if(this.velocity.isPresent()){
            // Compute velocity in local coordinates
            RealVector localVelocity = physicalVehicle.getRotation().transpose().operate(this.velocity.get());
            // Set initial velocity values
            ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("v_x_0", localVelocity.getEntry(0));
            ((ModelicaPhysicalVehicle) physicalVehicle).getVDM().setParameter("v_y_0", localVelocity.getEntry(1));
            // todo set wheel velocity

        }
        this.angularVelocity.ifPresent(physicalVehicle::setAngularVelocity);

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

        return physicalVehicle;
    }
}
