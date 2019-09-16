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
import org.apache.commons.math3.linear.RealVector;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Created by Aklima Zaman on 18-Dec-16.
 */

public class SpeedSensor extends AbstractSensor {

    private Double value;

    public SpeedSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                       HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator,subscribedMessages,targetsByMessageId);
    }

    @Override
    public Double getValue() {
        return this.value;
    }

    @Override
    public int getDataLength() {
        return 6;
    }

    /**
     * Calculated velocity cannot be negative
     */
    @Override
    protected void calculateValue() {
        RealVector velocity = getPhysicalVehicle().getVelocity().copy();
        double velocityValue = velocity.getNorm();
        //NormalDistribution normalDistribution = new NormalDistribution(velocityValue, 0.1);
        //velocityValue = normalDistribution.sample();
        while (velocityValue < 0) {
            //velocityValue = normalDistribution.sample();
        }

        this.value = new Double(velocityValue);
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_VELOCITY;
    }

    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_VELOCITY;
    }

    @Override
    public String getTypeName() {
        return Double.class.getTypeName();
    }
}
