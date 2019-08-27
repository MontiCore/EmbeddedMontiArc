package simulation.environment.util;

import simulation.environment.object.Battery;
import simulation.environment.object.ChargingStation;
import java.util.Optional;

public interface Chargeable {
    boolean isElectricVehicle();
    Optional<Battery> getBattery();
    //		boolean isFullyCharged();
    boolean isParkedChargingStation(ChargingStation station);
    void onRechargeReady();
}
