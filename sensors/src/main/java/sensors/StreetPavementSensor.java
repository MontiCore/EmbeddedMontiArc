package sensors;
import commons.controller.commons.BusEntry;
import commons.simulation.Sensor;
import sensors.abstractsensors.AbstractSensor;
import simulation.environment.World;
import simulation.environment.WorldModel;
import simulation.environment.geometry.osmadapter.GeomStreet;
import simulation.environment.visualisationadapter.implementation.Street2D;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
import sensors.factory.SensorFactory;
import simulation.vehicle.PhysicalVehicle;
import simulation.vehicle.Vehicle;
import static commons.controller.commons.BusEntry.SENSOR_STREETPAVEMENT;

import java.util.Optional;



/**
 * Created by Theresa on 08.01.2019.
 */


public class StreetPavementSensor extends AbstractSensor {

    private String value;

    public StreetPavementSensor(PhysicalVehicle physicalVehicle) {
        super(physicalVehicle);
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_STREETPAVEMENT;
    }

    @Override
    public String getTypeName() {
        return String.class.getTypeName();
    }

    @Override
    protected void calculateValue() {
        World world = WorldModel.getInstance();
        GeomStreet geom = world.getStreet(getPhysicalVehicle());
        EnvStreet env = (EnvStreet) geom.getObject();
        Street2D s2d = (Street2D) env;
        this.value = s2d.getStreetPavement().toString();
    }



    public double getfrictionceofficient() {
        Optional<Sensor> streetPavementSensor = this.getPhysicalVehicle().getSimulationVehicle().getSensorByType(getType());
        double frictioncoefficient = 1;
        if (streetPavementSensor.isPresent()) {
            String streetPavement = (String) (streetPavementSensor.get().getValue());

            switch (streetPavement) {
                case "PAVED":
                    frictioncoefficient = 1;
                    break;
                case "UNPAVED":
                    frictioncoefficient = 0.5;
                    break;
                case "QUALITY":
                    frictioncoefficient = 1;
                    break;
                case "STONE":
                    frictioncoefficient = 0.8;
                    break;
                case "DIRT":
                    frictioncoefficient = 0.5;
                    break;
                case "GRASS":
                    frictioncoefficient = 0.3;
                    break;
                default:
                    frictioncoefficient = 0.3;
                    break;
            }

        }

        return frictioncoefficient;
    }


    @Override
    public String getValue() {
        return this.value;
    }


}