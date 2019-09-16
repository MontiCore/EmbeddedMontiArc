/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.IPhysicalVehicle;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;

import java.util.HashMap;
import java.util.List;

/**
 * Created by Aklima Zaman on 20-Jan-17.
 */
public class SteeringAngleSensor extends AbstractSensor {

    private Double value;

    public SteeringAngleSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                               HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages,targetsByMessageId);
    }

    @Override
    public Double getValue() {
        return this.value;
    }

    @Override
    public int getDataLength() {
        return 6;
    }

    @Override
    protected void calculateValue() {
        //NormalDistribution normalDistribution = new NormalDistribution(tempValue, .001);
        //tempValue = DoubleMath.mean(normalDistribution.sample(10));
        this.value = this.getPhysicalVehicle().getSteeringAngle();
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_STEERING;
    }

    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_STEERING;
    }

    @Override
    public String getTypeName() {
        return Double.class.getTypeName();
    }
}
