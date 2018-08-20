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

        physicalVehicle.getSimulationVehicle().setControllerBus(controllerBus);
        physicalVehicle.getSimulationVehicle().setController(controller);
        physicalVehicle.getSimulationVehicle().setNavigation(navigation);

        physicalVehicle.initPhysics();

        return physicalVehicle;
    }
}
