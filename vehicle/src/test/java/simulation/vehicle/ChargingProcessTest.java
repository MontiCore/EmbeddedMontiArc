/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.interfaces.Bus;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.interfaces.FunctionBlockInterface;
import org.junit.*;
import static org.junit.Assert.*;

import org.powermock.api.mockito.PowerMockito;
import simulation.environment.util.ChargingProcess;
import simulation.environment.object.ChargingStation;
import simulation.environment.util.VehicleType;

import java.util.Optional;

/**
 * Charging Process Test Class
 *
 * @version 1.0
 * @since 2019-09-17
 */
public class ChargingProcessTest {

    @Test
    public void executeLoopIteration() {
        ChargingStation chargingStation = new ChargingStation();
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle(VehicleType.ELECTRIC,0.1);
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        vehicle.setController(Optional.of(PowerMockito.mock(FunctionBlockInterface.class)));
        vehicle.setControllerBus(Optional.of(PowerMockito.mock(Bus.class)));

        physicalVehicle.executeLoopIteration(10);

        ChargingProcess chargingProcess = new ChargingProcess(physicalVehicle,chargingStation);
        chargingProcess.startProcess();
        double timeToCharge = vehicle.getBattery().get().timeToCharge(100);


        // TODO
        long sysTime = System.currentTimeMillis();
        long timeToCharge2=0;
        while(vehicle.getBattery().get().getBatteryPercentage() != 100){
            chargingProcess.executeLoopIteration(3000);
            timeToCharge2 = timeToCharge2+2;

            if(System.currentTimeMillis() - sysTime > 60000){
                break;
            }
        }
        assertTrue(System.currentTimeMillis() - sysTime > 60000);

        /*
        while(timeToCharge >= 0){
            chargingProcess.executeLoopIteration(2000);
            timeToCharge = timeToCharge-2;
        }
        assertTrue(vehicle.getBattery().get().getBatteryPercentage() == 100);

         */
    }

    @Test
    public void startProcess() {
        ChargingStation chargingStation = new ChargingStation();
        PhysicalVehicle vehicle = new ModelicaPhysicalVehicle(VehicleType.ELECTRIC,20);

        ChargingProcess chargingProcess = new ChargingProcess(vehicle,chargingStation);
        chargingProcess.startProcess();
        assertTrue(vehicle.getBattery().get().getVoltageChargingStation() == 100);
        assertTrue(vehicle.getBattery().get().getAmpereChargingStation() == 1);
        assertTrue(vehicle.getBattery().get().getChargingStationConnectionStatus());
    }

    @Test
    public void stopProcess() {
        ChargingStation chargingStation = new ChargingStation();
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle(VehicleType.ELECTRIC,0.1);
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        vehicle.setController(Optional.of(PowerMockito.mock(FunctionBlockInterface.class)));
        vehicle.setControllerBus(Optional.of(PowerMockito.mock(Bus.class)));

        physicalVehicle.executeLoopIteration(10);
        assertTrue(vehicle.isGotoCharginstation());

        ChargingProcess chargingProcess = new ChargingProcess(physicalVehicle,chargingStation);
        chargingProcess.startProcess();
        chargingProcess.stopProcess();

        assertFalse(vehicle.getBattery().get().getChargingStationConnectionStatus());
        assertFalse(vehicle.isGotoCharginstation());
    }

}
