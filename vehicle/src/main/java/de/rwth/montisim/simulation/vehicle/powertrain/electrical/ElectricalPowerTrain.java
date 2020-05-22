/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;

import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.vehicleproperties.ElectricalPTProperties;

public class ElectricalPowerTrain extends PowerTrain {
    public final Battery battery;
    public final ElectricMotor e_motor;

    private final double transmissionRatio;

    public ElectricalPowerTrain(
        EESimulator ee_vehicle, ElectricalPTProperties properties,
        Class<? extends Battery> batteryClass, Class<? extends ElectricMotor> e_motorClass
    ) throws Exception {
        super("ElectricalPowerTrain");
        this.transmissionRatio = properties.transmissionRatio;
        try {
            Constructor<? extends Battery> batteryConstructor = batteryClass.getConstructor();
            this.battery = batteryConstructor.newInstance();
            this.battery.init(properties);

            Constructor<? extends ElectricMotor> mConstructor = e_motorClass.getConstructor();
            this.e_motor = mConstructor.newInstance();
            this.motor = this.e_motor;
            this.e_motor.setBattery(this.battery);
            this.e_motor.init(properties);
        } catch (NoSuchMethodException | SecurityException | 
        InstantiationException | IllegalAccessException | IllegalArgumentException
        | InvocationTargetException e) {
            //e.printStackTrace();
            throw new Exception(e);
        }
    }

    @Override
    public double getFuelPercentage() {
        return battery.percentage();
    }

    @Override
    public double getTransmissionRatio() {
        return transmissionRatio;
    }
}