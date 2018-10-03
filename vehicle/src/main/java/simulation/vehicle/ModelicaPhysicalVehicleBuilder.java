package simulation.vehicle;

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
    public PhysicalVehicle buildPhysicalVehicle(){
        PhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();

        this.controllerBus.ifPresent(physicalVehicle.getSimulationVehicle()::setControllerBus);
        this.controller.ifPresent(physicalVehicle.getSimulationVehicle()::setController);
        this.navigation.ifPresent(physicalVehicle.getSimulationVehicle()::setNavigation);
        this.mass.ifPresent(physicalVehicle::setMass);

        physicalVehicle.initPhysics();

        // todo set wheel velocity when setting the normal velocity

        // Compute velocity in local coordinates
        // RealVector localVelocity = rotation.transpose().operate(velocity);
        // Set initial velocity values
        // vehicleDynamicsModel.setParameter("v_x_0", localVelocity.getEntry(0));
        // vehicleDynamicsModel.setParameter("v_y_0", localVelocity.getEntry(1));

        return physicalVehicle;
    }
}
