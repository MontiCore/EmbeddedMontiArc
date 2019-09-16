/* (c) https://github.com/MontiCore/monticore */
package sensors;

/**
 * Created by Johannes on 11.07.2017.
 */
import commons.controller.commons.BusEntry;
import commons.simulation.IPhysicalVehicle;
import sensors.abstractsensors.AbstractDistanceSensor;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.environment.World;
import simulation.environment.WorldModel;

import java.util.HashMap;
import java.util.List;


public class RightBackWheelDistanceToStreetSensor extends AbstractDistanceSensor{
    public RightBackWheelDistanceToStreetSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                                                HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages,targetsByMessageId);
    }

    @Override
    protected Double calculateDistance(IPhysicalVehicle physicalVehicle) {
        World world = WorldModel.getInstance();
        double calculatedValue = world.getDistanceBackRightWheelToRightStreetBorder(physicalVehicle).doubleValue();
        //NormalDistribution normalDistribution = new NormalDistribution(calculatedValue, 0.01);
        return calculatedValue;
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR;
    }
    
    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR;
    }
}
