/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.environment.util;

import org.apache.commons.math3.linear.RealVector;
import simulation.environment.object.ChargingStation;
import java.util.Optional;

public interface Chargeable {
    boolean isCharging();
    void setCharging(boolean isCharging);
    boolean isChargeable();
    VehicleType getVehicleType();
    Optional<IBattery> getBattery();
    //		boolean isFullyCharged();
    boolean isParkedChargingStation(ChargingStation station);
    void onRechargeReady();
    RealVector getPosition();
}
