/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.util;

import de.rwth.montisim.simulation.environment.object.ChargingStation;

import java.time.Duration;
import java.util.Optional;

import de.rwth.montisim.commons.simulation.Updatable;

/**
 * Charging Station Class
 *
 * @version 1.0
 * @since 2019-05-22
 */
public class ChargingProcess implements Updatable {
    private Chargeable vehicle;
    private ChargingStation chargingStation;
    private Optional<IBattery> battery;
    private long chargingTimeMillis;

    // if true the chargingStation process is running
    private boolean chargeCar = false;

    public ChargingProcess(Chargeable vehicle, ChargingStation cs) {
        this.vehicle = vehicle;
        this.chargingStation = cs;
        this.battery = Optional.of(vehicle.getBattery().get());
    }

    public void startProcess() {

        // Battery Charging Protocol
        this.battery.get().setVoltageChargingStation(this.chargingStation.getVoltage());
        this.battery.get().setAmpereChargingStation(this.chargingStation.getAmpere());
        this.battery.get().connectToChargingStation();

        this.chargeCar = true;
    }

    public boolean stopProcess() {
        this.chargeCar = false;

        // Here can added more in case the chargingStation process ends
        this.battery.get().disconnectFromChargingStation();
        this.vehicle.onRechargeReady();
        this.chargingStation.stopCharging(vehicle);
        return chargeCar;
    }

    public Chargeable getVehicle() {
        return vehicle;
    }

    public void setVehicle(Chargeable vehicle) {
        this.vehicle = vehicle;
    }

    public ChargingStation getCs() {
        return chargingStation;
    }

    public void setCs(ChargingStation cs) {
        this.chargingStation = cs;
    }

    @Override
    public void update(Duration deltaT) {
        if (this.chargeCar) {

            // Car drives away?
            if (!chargingStation.carStandingAtTheCS(vehicle.getDynamicObject())) {
                stopProcess();
            }

            if (this.battery.get().getBatteryPercentage() > 99d) {
                stopProcess();
                return;
            }

            double consumtionOfThisLoop = this.battery.get().charge();
            // Increased consumption through transaction
            double IncreaseConsumptionPercent = 5;
            consumtionOfThisLoop = (5 * consumtionOfThisLoop / 100) + consumtionOfThisLoop;

            // Update Charging Station consumtion
            this.chargingStation.setConsumption(this.chargingStation.getConsumption() + consumtionOfThisLoop);
        }
    }
}
