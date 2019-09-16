/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors.abstractsensors;

import commons.controller.commons.BusEntry;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import commons.simulation.IPhysicalVehicle;
import java.util.HashMap;
import java.util.List;

/**
 * Created by Aklima Zaman on 2/8/2017.
 */
public abstract class AbstractDistanceSensor extends AbstractSensor {
    private Double value;


    public AbstractDistanceSensor(IPhysicalVehicle phyiscalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                                  HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(phyiscalVehicle, simulator, subscribedMessages, targetsByMessageId);
    }


	@Override
    protected void calculateValue() {
        this.value = calculateDistance(this.getPhysicalVehicle());
    }



    protected abstract Double calculateDistance(IPhysicalVehicle phyiscalVehicle);

    @Override
    public Double getValue() {
        return this.value;
    }

    @Override
    public int getDataLength() {
        return 6;
    }

    @Override
    public String getTypeName() {
        return Double.class.getTypeName();
    }
}
