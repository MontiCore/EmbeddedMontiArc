/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.util;

import de.rwth.montisim.simulation.environment.object.ChargingStation;

import java.util.Optional;

import de.rwth.montisim.commons.simulation.DynamicObject;

public interface Chargeable {
    boolean isCharging();

    void setCharging(boolean isCharging);

    boolean isChargeable();

    VehicleType getVehicleType();

    Optional<IBattery> getBattery();

    //		boolean isFullyCharged();
    boolean isParkedChargingStation(ChargingStation station);

    void onRechargeReady();

    DynamicObject getDynamicObject();
}
