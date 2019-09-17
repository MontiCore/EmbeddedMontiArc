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

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;

import java.util.HashMap;
import java.util.List;

/**
 * Created by Aklima Zaman on 20-Jan-17.
 */
public class LocationSensor extends AbstractSensor {

    private RealVector value;

    public LocationSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                          HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages,targetsByMessageId);
        value = new ArrayRealVector();
    }

    @Override
    public RealVector getValue() {
        return this.value;
    }

    @Override
    public int getDataLength() {
        return 15;
    }

    @Override
    protected void calculateValue() {
        RealVector pos = getPhysicalVehicle().getPosition().copy();
        double x_axis = pos.getEntry(0);
        double y_axis = pos.getEntry(1);
        double z_axis = pos.getEntry(2); // we are ignoring z axis for now

        //NormalDistribution normalDistribution_x = new NormalDistribution(x_axis, 0.1);
        //x_axis = normalDistribution_x.sample();

        //NormalDistribution normalDistribution_y = new NormalDistribution(y_axis, 0.1);
        //y_axis = normalDistribution_y.sample();

        pos.setEntry(0, x_axis);
        pos.setEntry(1, y_axis);
        pos.setEntry(2, z_axis);

        this.value = pos;

    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_GPS_COORDINATES;
    }

    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_GPS_COORDINATES;
    }

    @Override
    public String getTypeName() {
        return RealVector.class.getTypeName();
    }
}
