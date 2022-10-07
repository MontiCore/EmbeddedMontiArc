/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.actuator;

import java.util.logging.Logger;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValueDouble;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorLogic;

public class Actuator extends EEComponent implements Updatable {
    public static final String SETTER_PREFIX = "set_";

    public final transient ActuatorProperties properties;
    final transient PhysicalValueDouble actuatedValue;
    SensorLogic sensor;
    public double targetValue;
    final transient boolean sendFeedback; // Whether the actuator senses itself and sends its current value on the bus. ~>
    // "Sensor enabled"
    //transient boolean isDouble;
    transient int msgId;

    public Actuator(ActuatorProperties properties, EESystem eesystem, PhysicalValueDouble actuatedValue, Updater updater)
            throws EEMessageTypeException {
        super(properties, eesystem);
        // isDouble = actuatedValue.type.equals(BasicType.DOUBLE);
        // if (!isDouble && !actuatedValue.type.equals(BasicType.FLOAT))
        //     throw new IllegalArgumentException("Actuator can only actuate on float or double values (but here type "
        //             + actuatedValue.type + " for value " + actuatedValue.name + ")");
        this.properties = properties;
        this.actuatedValue = actuatedValue;
        this.targetValue = actuatedValue.getValue();
        // this.targetValue = isDouble ? (Double) actuatedValue.get()
        //         : (Float) actuatedValue.get();
        updater.addUpdatable(this);
        this.sendFeedback = properties.sensorProperties.isPresent();
        if (sendFeedback)
            this.sensor = new SensorLogic(this, properties.sensorProperties.get(), actuatedValue);

        this.msgId = addPort(PortInformation.newOptionalInputDataPort(SETTER_PREFIX + actuatedValue.name, BasicType.DOUBLE, true));
    }


    @Override
    public void update(TimeUpdate newTime) {
        actuate(newTime);
        if (sendFeedback)
            sensor.update(newTime);
    }

    // Can be overwritten for more complex actuation behavior
    protected void actuate(TimeUpdate newTime) {
        // double val = isDouble ? (Double) actuatedValue.get()
        //         : (Float) actuatedValue.get();
        double val = actuatedValue.getValue();
        double maxChange = properties.change_rate * newTime.deltaSeconds;
        double delta = targetValue - val;
        if (Math.abs(delta) > maxChange) {
            delta = Math.signum(delta) * maxChange;
        }
        val += delta;
        if (val > actuatedValue.getMax())
            val = actuatedValue.getMax();
        if (val < actuatedValue.getMin())
            val = actuatedValue.getMin();
        //actuatedValue.set(isDouble ? Double.valueOf(val) : Float.valueOf((float)val));
        actuatedValue.set(Double.valueOf(val));
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        if (msg.isMsg(msgId)) {
            targetValue = (Double) msg.message;
        } else {
            Logger.getLogger("Warnings").warning("Actuator \"" + properties.name + "\" received unexpected message: "
                    + msg.msgInfo.name);
        }
    }

}