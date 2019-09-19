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
import org.apache.commons.math3.distribution.NormalDistribution;
import simulation.environment.World;
import simulation.environment.WorldModel;
import simulation.vehicle.PhysicalVehicle;

/**
 * Created by Zaman on 2/8/2017.
 */
public class DistanceToLeftSensor extends AbstractDistanceSensor {

    public DistanceToLeftSensor(PhysicalVehicle vehicle) {
        super(vehicle);
    }

    @Override
    protected Double calculateDistance(IPhysicalVehicle v) {
        World world = WorldModel.getInstance();
        double calculatedValue = world.getDistanceToLeftStreetBorder(v).doubleValue();
        NormalDistribution normalDistribution = new NormalDistribution(calculatedValue, 0.01);
        return calculatedValue;//new Double(normalDistribution.sample());
    }

    @Override
    public BusEntry getType() {

        return BusEntry.SENSOR_DISTANCE_TO_LEFT;
    }
}
