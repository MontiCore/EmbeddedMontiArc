/* (c) https://github.com/MontiCore/monticore */
package sensors;

import commons.controller.commons.BusEntry;
import commons.simulation.IPhysicalVehicle;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;

import java.util.HashMap;
import java.util.List;

/**
 * Created by Johannes on 12.09.2017.
 */
public class DayNightSensor extends AbstractSensor {
    public enum Daytime{
        Day,Night
    }

    private Daytime value;

    public DayNightSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                          HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages, targetsByMessageId);
    }

    @Override
    protected void calculateValue() {
        this.value = Daytime.Day;
    }

    @Override
    public Daytime getValue() {
        return this.value;
    }

    @Override
    public int getDataLength() {
        return 1;
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_DAYNIGHT;
    }
    
    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_DAYNIGHT;
    }

    @Override
    public String getTypeName() {
        return Daytime.class.getTypeName();
    }
}
