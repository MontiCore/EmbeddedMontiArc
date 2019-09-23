/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors;

import org.apache.commons.math3.distribution.NormalDistribution;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import sensors.abstractsensors.AbstractSensor;
import simulation.environment.World;
import simulation.environment.WorldModel;
import simulation.vehicle.PhysicalVehicle;

/**
 * Created by zaman on 2/8/2017.
 */
public class WeatherSensor extends AbstractSensor {
    private Double value;

    public WeatherSensor(PhysicalVehicle physicalVehicle) {
        super(physicalVehicle);
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
    public BusEntry getType() {
        return BusEntry.SENSOR_WEATHER;
    }

    @Override public String getTypeName() {
        return Double.class.getTypeName();
    }
}
