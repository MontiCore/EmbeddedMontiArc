/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.IPhysicalVehicle;
import sensors.abstractsensors.AbstractDistanceSensor;
import simulation.environment.World;
import simulation.environment.WorldModel;
import simulation.vehicle.PhysicalVehicle;

/**
 * Created by Marius on 12.09.2017.
 */
public class LeftFrontDistanceSensor extends AbstractDistanceSensor {

    public LeftFrontDistanceSensor(PhysicalVehicle vehicle) {
        super(vehicle);
    }

    @Override
    protected Double calculateDistance(IPhysicalVehicle v) {
        World world = WorldModel.getInstance();
        double calculatedValue = world.getDistanceLeftFrontToStreetBorder(v).doubleValue();
        //NormalDistribution normalDistribution = new NormalDistribution(calculatedValue, 0.01);
        return calculatedValue;
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_LEFT_FRONT_DISTANCE;
    }
}
