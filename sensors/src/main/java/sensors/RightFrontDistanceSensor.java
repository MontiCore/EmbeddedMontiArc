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
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.environment.World;
import simulation.environment.WorldModel;

import java.util.HashMap;
import java.util.List;

/**
 * Created by Marius on 12.09.2017.
 */
public class RightFrontDistanceSensor extends AbstractDistanceSensor {

    public RightFrontDistanceSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                                    HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages, targetsByMessageId);
    }

    @Override
    protected Double calculateDistance(IPhysicalVehicle physicalVehicle) {
        World world = WorldModel.getInstance();
        double calculatedValue = world.getDistanceRightFrontToStreetBorder(physicalVehicle).doubleValue();
        //NormalDistribution normalDistribution = new NormalDistribution(calculatedValue, 0.01);
        return calculatedValue;
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_RIGHT_FRONT_DISTANCE;
    }

    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_RIGHT_FRONT_DISTANCE;
    }
}
