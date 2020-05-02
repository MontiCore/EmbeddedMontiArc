/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.actuator;

import java.time.Duration;
import java.util.logging.Logger;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorLogic;

public class Actuator extends EEComponent implements Updatable {
    final SensorLogic sensor;
    double targetValue;
    final PhysicalValue currentValue;
    final double minValue;
    final double maxValue;
    double changeRate; // Max "value per second" this actuator can actuate.
    final boolean sendFeedback; // Whether the actuator senses itself and sends its current value on the bus. ~> "Sensor enabled"
    
    final MessageInformation msgInfo;

    public Actuator(
        EESimulator simulator, String name, int priority, 
        PhysicalValue actuatedValue, double minValue, double maxValue, double changeRate,
        boolean sendFeedback, Duration updateInterval, Duration readTime, boolean sendOnlyChanged,
        Updater updater
    ){
        super(simulator, name, priority);
        sensor = new SensorLogic(actuatedValue, updateInterval, readTime, sendOnlyChanged, this);
        this.currentValue = actuatedValue;
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.targetValue = actuatedValue.get();
        this.changeRate = changeRate;
        this.sendFeedback = sendFeedback;
        this.msgInfo = addInput("set_"+actuatedValue.name, DataType.newDoubleType());
        updater.addUpdatable(this);
    }

    /** Returns an actuator which doesn't send its value back. */
    public static Actuator newSimpleActuator(
        EESimulator simulator, String name, int priority,
        PhysicalValue actuatedValue, double minValue, double maxValue, double changeRate,
        Updater updater
    ){
        return new Actuator(simulator, name, priority, actuatedValue, minValue, maxValue, changeRate, false, Duration.ZERO, Duration.ZERO, true, updater);
    }

    @Override
    public void update(TimeUpdate newTime) {
        actuate(newTime);
        if (sendFeedback)
            sensor.update(newTime);
    }

    // Can be overwritten for more complex actuation behavior
    protected void actuate(TimeUpdate newTime){
        double val = currentValue.get();
        double maxChange = changeRate*newTime.deltaSeconds;
        double delta = targetValue - val;
        if (Math.abs(delta) > maxChange){
            delta = Math.signum(delta) * maxChange;
        }
        val += delta;
        if (val > maxValue)
            val = maxValue;
        if (val < minValue)
            val = minValue;
        currentValue.set(val);
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        if (msg.msgId == msgInfo.messageId){
            targetValue = (Double)msg.message;
        } else {
            Logger.getLogger("Warnings").warning("Actuator \""+name+"\" received unexpected message: " + simulator.getMessageTypeManager().getMsgInfo(msg.msgId).name);
        }
    }

    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.ACTUATOR;
    }

}