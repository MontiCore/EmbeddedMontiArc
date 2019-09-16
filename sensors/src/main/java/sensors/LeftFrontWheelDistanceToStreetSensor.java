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
 * Created by Johannes on 07.07.2017.
 */
public class LeftFrontWheelDistanceToStreetSensor extends AbstractDistanceSensor{
    public LeftFrontWheelDistanceToStreetSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                                                HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages,targetsByMessageId);
    }

    @Override
    protected Double calculateDistance(IPhysicalVehicle physicalVehicle) {
        World world = WorldModel.getInstance();
        double calculatedValue = world.getDistanceFrontLeftWheelToLeftStreetBorder(physicalVehicle).doubleValue();
        //NormalDistribution normalDistribution = new NormalDistribution(calculatedValue, 0.01);
        return calculatedValue;
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR;
    }
    
    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR;
    }
}
