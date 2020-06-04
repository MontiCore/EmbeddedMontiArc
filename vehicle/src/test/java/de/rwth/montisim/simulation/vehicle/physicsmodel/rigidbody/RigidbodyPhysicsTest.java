/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody;

import java.time.*;
import org.junit.*;

import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.vehicle.*;
import de.rwth.montisim.simulation.vehicle.config.TestVehicleConfig;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties.BatteryType;

public class RigidbodyPhysicsTest {

    @Test
    public void testAccel() throws Exception {
        TestVehicleConfig config = new TestVehicleConfig();
        config.electricalPTProperties.batteryProperties = new BatteryProperties(BatteryType.SIMPLE);

        MessageTypeManager mtManager = new MessageTypeManager();
        Vehicle v = new VehicleBuilder(mtManager, null, config).setName("TestVehicle").build();
        
        v.physicsModel.setGroundPosition(new Vec3(0,0,0), new Vec2(1,0));

        v.powerTrain.gasValue.set(1.0);

        RigidbodyPhysics physics = (RigidbodyPhysics) v.physicsModel;

        long ST = System.nanoTime();
        TimeUpdate tu = new TimeUpdate(Instant.EPOCH, Duration.ofMillis(10)); // Don't update absolute time -> physics should only use deltas
        for (int i = 0; i < 1000; ++i){
            v.physicsModel.update(tu);
        }

        Assert.assertTrue("Vehicle did not move", physics.rb.pos.x > 100);
        Assert.assertTrue("Vehicle velocity too small", physics.rb.velocity.x > 50);

        // long ET = System.nanoTime();
        // Logger.getGlobal().info("1000 physics steps: "+((ET-ST)*0.000001)+" ms.");
        // Logger.getGlobal().info("Pos: "+physics.rb.pos);
        // Logger.getGlobal().info("Vel: "+physics.rb.velocity);
    }
}