/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.IPhysicalVehicle;
import org.apache.commons.math3.distribution.NormalDistribution;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.environment.World;
import simulation.environment.WorldModel;

import java.util.HashMap;
import java.util.List;

/**
 * Created by zaman on 2/8/2017.
 */
public class WeatherSensor extends AbstractSensor {
    private Double value;

    public WeatherSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                         HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages,targetsByMessageId);
    }

    @Override
    protected void calculateValue() {
        World world = WorldModel.getInstance();
        double weatherValue = world.getWeather();
        NormalDistribution normalDistribution = new NormalDistribution(weatherValue, 0.0003);
        this.value = new Double(normalDistribution.sample());
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
    public BusEntry getType() {
        return BusEntry.SENSOR_WEATHER;
    }

    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_WEATHER;
    }

    @Override public String getTypeName() {
        return Double.class.getTypeName();
    }
}
