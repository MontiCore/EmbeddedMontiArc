/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.actuator;

import java.util.logging.Logger;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.dynamicinterface.DataType.Type;
import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorLogic;

public class Actuator extends EEComponent implements Updatable {
    public static final String SETTER_PREFIX = "set_";

    public final transient ActuatorProperties properties;
    final transient PhysicalValue actuatedValue;
    SensorLogic sensor;
    public double targetValue;
    final transient boolean sendFeedback; // Whether the actuator senses itself and sends its current value on the bus. ~>
                                // "Sensor enabled"

    transient MessageInformation msgInfo;

    public Actuator(ActuatorProperties properties, PhysicalValue actuatedValue, Updater updater) {
        super(properties);
        if (actuatedValue.type.type != Type.DOUBLE && actuatedValue.type.type != Type.FLOAT)
            throw new IllegalArgumentException("Actuator can only actuate on float or double values (but here type "
                    + actuatedValue.type + " for value " + actuatedValue.name + ")");
        this.properties = properties;
        this.actuatedValue = actuatedValue;
        this.targetValue = actuatedValue.type.type == Type.DOUBLE ? (Double) actuatedValue.get()
                : (Float) actuatedValue.get();
        updater.addUpdatable(this);
        this.sendFeedback = properties.sensorProperties.isPresent();
        if (sendFeedback)
            this.sensor = new SensorLogic(properties.sensorProperties.get(), actuatedValue);
    }

    @Override
    protected void init() throws EEMessageTypeException {
        this.msgInfo = addOptionalInput(SETTER_PREFIX + actuatedValue.name, DataType.DOUBLE, true);
        if (sendFeedback)
            sensor.init(this);
    }

    @Override
    public void update(TimeUpdate newTime) {
        actuate(newTime);
        if (sendFeedback)
            sensor.update(newTime);
    }

    // Can be overwritten for more complex actuation behavior
    protected void actuate(TimeUpdate newTime) {
        double val = actuatedValue.type.type == Type.DOUBLE ? (Double) actuatedValue.get()
                : (Float) actuatedValue.get();
        double maxChange = properties.change_rate * newTime.deltaSeconds;
        double delta = targetValue - val;
        if (Math.abs(delta) > maxChange) {
            delta = Math.signum(delta) * maxChange;
        }
        val += delta;
        if (val > properties.max)
            val = properties.max;
        if (val < properties.min)
            val = properties.min;
        actuatedValue.set(actuatedValue.type.type == Type.DOUBLE ? new Double(val) : new Float(val));
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        if (msg.isMsg(msgInfo)) {
            targetValue = (Double) msg.message;
        } else {
            Logger.getLogger("Warnings").warning("Actuator \"" + properties.name + "\" received unexpected message: "
                    + msg.msgInfo.name);
        }
    }

    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.ACTUATOR;
    }

}