/* (c) https://github.com/MontiCore/monticore */
package sensors;

import commons.controller.commons.BusEntry;
import commons.simulation.IPhysicalVehicle;
import commons.simulation.PhysicalObject;
import commons.simulation.Sensor;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import org.apache.commons.lang3.Validate;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.EESimulator;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;


public abstract class StaticPlannedTrajectorySensor extends  AbstractSensor {

    private final List<Double> trajectory;

    public StaticPlannedTrajectorySensor(IPhysicalVehicle phyiscalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
            HashMap<BusEntry, List<EEComponent>> targetsByMessageId, List<Double> trajectory) {
        super(phyiscalVehicle,simulator, subscribedMessages, targetsByMessageId);
        Validate.notNull(trajectory);
        List<Double> defensiveCopy = new ArrayList<>(trajectory);
        Validate.noNullElements(defensiveCopy);
        this.trajectory = Collections.unmodifiableList(defensiveCopy);
    }

    @Override
    public Object getValue() {
        return trajectory;
    }

    @Override
    public String getTypeName() {
        return Collections.<Double>emptyList().getClass().getTypeName();
    }

    @Override
    public void update(Instant actualTime) {
    }

    @Override
    public int getDataLength(){
        return 8 * trajectory.size();
    }
}
