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
        this.mass.ifPresent(physicalVehicle.getSimulationVehicle()::setMass);

        physicalVehicle.initPhysics();

        return physicalVehicle;
    }
}
