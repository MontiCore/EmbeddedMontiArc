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
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;

import java.util.HashMap;
import java.util.List;

/**
 * Created by kirchhof on 10/03/2017.
 */
public class CompassSensor extends AbstractSensor {

    private Double value;

    public CompassSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                         HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages, targetsByMessageId);
    }

    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_COMPASS;
    }

	public static BusEntry getSensorType() {
        return BusEntry.SENSOR_COMPASS;
	}

    @Override
    public Object getValue() {
        return value;
    }

    @Override
    public int getDataLength() {
        return 6;
    }

    @Override
    public String getTypeName() {
        return Double.class.getTypeName();
    }

    @Override
    protected void calculateValue() {
        RealMatrix rotation = getPhysicalVehicle().getRotation();
        RealVector yAxis = new ArrayRealVector(new double[] {0.0, 1.0, 0.0});
        RealVector v = rotation.operate(yAxis);
        //the angle in the xy-plane is needed
        v.setEntry(2, 0.0);

        double cos = v.cosine(yAxis);
        double angle = Math.acos(cos);

        //If the vector points to quadrant I or IV, ensure value range goes up to 2*pi
        if (v.getEntry(0) > 0) {
            angle = -angle;
            angle += 2*Math.PI;
        }
        value = angle;
    }




    }
