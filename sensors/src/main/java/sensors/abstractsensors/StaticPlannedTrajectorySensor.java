/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package sensors.abstractsensors;


import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.IPhysicalVehicle;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import org.apache.commons.lang3.Validate;

import java.time.Instant;
import java.util.*;


public abstract class StaticPlannedTrajectorySensor extends  AbstractSensor {

    private List<Double> trajectory;

    public StaticPlannedTrajectorySensor(IPhysicalVehicle phyiscalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                                         HashMap<BusEntry, List<EEComponent>> targetsByMessageId, List<Double> trajectory) {
        super(phyiscalVehicle,simulator, subscribedMessages, targetsByMessageId);
        Validate.notNull(trajectory);
        List<Double> defensiveCopy = new ArrayList<>(trajectory);
        Validate.noNullElements(defensiveCopy);
        this.trajectory = Collections.unmodifiableList(defensiveCopy);
    }

    public StaticPlannedTrajectorySensor(IPhysicalVehicle phyiscalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                                         HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(phyiscalVehicle,simulator, subscribedMessages, targetsByMessageId);
    }

    public void initializeTrajectory(List<Double> trajectoryX) {
        Validate.notNull(this.trajectory);
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
