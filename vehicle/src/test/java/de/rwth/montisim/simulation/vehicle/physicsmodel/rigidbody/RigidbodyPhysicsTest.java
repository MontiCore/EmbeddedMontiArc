/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody;

import java.time.Duration;
import java.time.Instant;
import java.util.logging.Logger;

import org.junit.Test;

import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.vehicle.vehicleproperties.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.vehicleproperties.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricMotor;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.SimpleBattery;

public class RigidbodyPhysicsTest {

    @Test
    public void testAccel() throws Exception {
        ElectricalPTProperties eptp = new ElectricalPTProperties();
        VehicleProperties properties = new VehicleProperties(eptp);
        EESimulator eesimulator = new EESimulator();
        RigidbodyPhysics physics = new RigidbodyPhysics(properties, new ElectricalPowerTrain(eesimulator, eptp, SimpleBattery.class, ElectricMotor.class), eesimulator, new Updater());
        physics.setGroundPosition(new Vec3(0,0,0), new Vec2(1,0));

        physics.gasValue.set(1);

        long ST = System.nanoTime();
        TimeUpdate tu = new TimeUpdate(Instant.EPOCH, Duration.ofMillis(10)); // Don't update absolute time -> physics should only use deltas
        for (int i = 0; i < 1000; ++i){
            physics.update(tu);
        }
        long ET = System.nanoTime();
        Logger.getGlobal().info("1000 physics steps: "+((ET-ST)*0.000001)+" ms.");
        Logger.getGlobal().info("Pos: "+physics.rb.pos);
        Logger.getGlobal().info("Vel: "+physics.rb.velocity);
    }
}