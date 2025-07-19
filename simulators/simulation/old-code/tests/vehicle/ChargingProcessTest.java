/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.montisim.commons.controller.commons.BusEntry;
import de.rwth.montisim.commons.controller.interfaces.Bus;
import de.rwth.montisim.commons.controller.interfaces.FunctionBlockInterface;
import de.rwth.monticore.EmbeddedMontiArc.simulators.controller.library.databus.DataBus;
import org.junit.*;

import static org.junit.Assert.*;

import org.powermock.api.mockito.PowerMockito;
import de.rwth.montisim.simulation.environment.util.ChargingProcess;
import de.rwth.montisim.simulation.environment.object.ChargingStation;
import de.rwth.montisim.simulation.environment.util.VehicleType;

import java.util.Optional;

import simulation.EESimulator.EESimulator;
import simulation.bus.InstantBus;

import java.time.Duration;
import java.time.Instant;

/**
 * Charging Process Test Class
 *
 * @version 1.0
 * @since 2019-09-17
 */
public class ChargingProcessTest {

    private Vehicle createStandardVehicle(PhysicalVehicleBuilder physicalVehicleBuilder) {
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        return new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
    }

    @Test
    public void executeLoopIteration() throws Exception {
        ChargingStation chargingStation = new ChargingStation();
        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        vehicle.setVehicleType(VehicleType.ELECTRIC, 0.1);

        Battery battery = new Battery(vehicle, 10000, 50);
        battery.set_local_delta_t(33d);
        vehicle.setBattery(battery);
        vehicle.getPhysicalVehicle().executeLoopIteration(Duration.ofMillis(10));

        ChargingProcess chargingProcess = new ChargingProcess(vehicle.getPhysicalVehicle(), chargingStation);
        chargingProcess.startProcess();
        double timeToCharge = vehicle.getBattery().get().timeToCharge(100);

        while (timeToCharge >= 0) {
            chargingProcess.executeLoopIteration(Duration.ofMillis(33));
            timeToCharge--;
        }
        assertTrue(vehicle.getBattery().get().getBatteryPercentage() == 100);

    }

    @Test
    public void startProcess() throws Exception {
        ChargingStation chargingStation = new ChargingStation();
        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        vehicle.setVehicleType(VehicleType.ELECTRIC, 20);

        ChargingProcess chargingProcess = new ChargingProcess(vehicle.getPhysicalVehicle(), chargingStation);
        chargingProcess.startProcess();
        assertTrue(vehicle.getBattery().get().getVoltageChargingStation() == 100);
        assertTrue(vehicle.getBattery().get().getAmpereChargingStation() == 1);
        assertTrue(vehicle.getBattery().get().getChargingStationConnectionStatus());
    }

    @Test
    public void stopProcess() throws Exception {
        ChargingStation chargingStation = new ChargingStation();
        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        vehicle.setVehicleType(VehicleType.ELECTRIC, 0.1);

        vehicle.getPhysicalVehicle().executeLoopIteration(Duration.ofMillis(10));
        assertTrue(vehicle.isGotoCharginstation());

        ChargingProcess chargingProcess = new ChargingProcess(vehicle.getPhysicalVehicle(), chargingStation);
        chargingProcess.startProcess();
        chargingProcess.stopProcess();

        assertFalse(vehicle.getBattery().get().getChargingStationConnectionStatus());
        assertFalse(vehicle.isGotoCharginstation());
    }

}
