/* (c) https://github.com/MontiCore/monticore */
package sensors;

/**
 * Created by Johannes on 11.07.2017.
 */
import commons.controller.commons.BusEntry;
import commons.simulation.IPhysicalVehicle;
import sensors.abstractsensors.AbstractDistanceSensor;
import simulation.environment.World;
import simulation.environment.WorldModel;
import simulation.vehicle.PhysicalVehicle;


public class RightFrontWheelDistanceToStreetSensor extends AbstractDistanceSensor{
    public RightFrontWheelDistanceToStreetSensor(PhysicalVehicle vehicle) {
        super(vehicle);
    }

    @Override
    protected Double calculateDistance(IPhysicalVehicle v) {
        World world = WorldModel.getInstance();
        double calculatedValue = world.getDistanceFrontRightWheelToRightStreetBorder(v).doubleValue();
        //NormalDistribution normalDistribution = new NormalDistribution(calculatedValue, 0.01);
        return calculatedValue;
    }

    @Override
    public BusEntry getType() {

        return BusEntry.SENSOR_RIGHT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR;
    }
}
