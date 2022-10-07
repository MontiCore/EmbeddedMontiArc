/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody;

import java.time.*;

import org.junit.*;

import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.vehicle.*;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery.BatteryProperties.BatteryType;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor.ElectricMotorProperties;

public class RigidbodyPhysicsTest {

    /**
     * Accelerates the vehicle at full throttle in the +x direction.
     * Simulates at 10ms ticks for 1000 ticks (-> 10 seconds)
     */
    @Test
    public void testAccel() throws Exception {
        VehicleProperties properties = new VehicleProperties();
        ElectricalPowerTrain powerTrain = new ElectricalPowerTrain(new ElectricalPTProperties(new ElectricMotorProperties(), new BatteryProperties(BatteryType.SIMPLE)));

        RigidbodyPhysics physics = new RigidbodyPhysics(new RigidbodyPhysicsProperties(), powerTrain, properties);

        physics.setGroundPosition(new Vec3(0, 0, 0), new Vec2(1, 0));
        powerTrain.gasValue.set(1.0);
        powerTrain.brakingValue.set(0.0);
        powerTrain.steeringValue.set(0.0);

        //long ST = System.nanoTime();
        TimeUpdate tu = new TimeUpdate(Instant.EPOCH, Duration.ofMillis(10)); // Don't update absolute time -> physics should only use deltas
        for (int i = 0; i < 1000; ++i) {
            physics.update(tu);
        }

        String state = " Pos: " + physics.rb.pos + " Vel: " + physics.rb.velocity;
        Assert.assertTrue("Vehicle did not move: " + state, physics.rb.pos.x > 100);
        Assert.assertTrue("Vehicle position too far: " + state, physics.rb.pos.x < 500);
        Assert.assertTrue("Vehicle position off track: " + state, physics.rb.pos.y < 1 && physics.rb.pos.y > -1);

        Assert.assertTrue("Vehicle velocity too small: " + state, physics.rb.velocity.x > 50);
        Assert.assertTrue("Vehicle velocity too big: " + state, physics.rb.velocity.magnitude() < 150);
        Assert.assertTrue("Vehicle velocity off track: " + state, physics.rb.velocity.y < 1 && physics.rb.velocity.y > -1);


        // long ET = System.nanoTime();
        // Logger.getGlobal().info("1000 physics steps: "+((ET-ST)*0.000001)+" ms.");
        // Logger.getGlobal().info("Pos: "+physics.rb.pos);
        // Logger.getGlobal().info("Vel: "+physics.rb.velocity);
    }

    // TODO test braking
}