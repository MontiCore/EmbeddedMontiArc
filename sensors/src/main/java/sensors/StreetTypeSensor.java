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
import simulation.bus.BusMessage;
import simulation.environment.World;
import simulation.environment.WorldModel;
import simulation.environment.geometry.osmadapter.GeomStreet;
import simulation.environment.visualisationadapter.implementation.Street2D;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;

import java.time.Instant;
import java.util.HashMap;
import java.util.List;

/**
 * Created by Henk on 04.08.2017.
 */
public class StreetTypeSensor extends AbstractSensor {

    private String value;

    public StreetTypeSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
                            HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(physicalVehicle, simulator, subscribedMessages,targetsByMessageId);
        value = "";
    }


    @Override
    public BusEntry getType() {
        return BusEntry.SENSOR_STREETTYPE;
    }

    public static BusEntry getSensorType() {
        return BusEntry.SENSOR_STREETTYPE;
    }

    @Override
    public int getDataLength() {
        return (value.length() * 2);
    }

    @Override
    protected void calculateValue() {
        World world = WorldModel.getInstance();
        GeomStreet geom = world.getStreet(getPhysicalVehicle());
        EnvStreet env = (EnvStreet) geom.getObject();
        Street2D s2d = (Street2D) env;
        this.value = s2d.getStreetType().toString();
    }

    @Override
    public String getValue() {
        return this.value;
    }

    @Override
    public String getTypeName() {
        return String.class.getTypeName();
    }
}
