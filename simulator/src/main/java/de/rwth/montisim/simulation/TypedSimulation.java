/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation;

import de.rwth.montisim.commons.TypedCommons;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.simulation.eecomponents.autopilots.*;
import de.rwth.montisim.simulation.eecomponents.lidar.LidarProperties;
import de.rwth.montisim.simulation.eecomponents.simple_network.*;
import de.rwth.montisim.simulation.eecomponents.speed_limit.SpeedLimitServiceProperties;
import de.rwth.montisim.simulation.eesimulator.actuator.ActuatorProperties;
import de.rwth.montisim.simulation.eesimulator.bridge.BridgeProperties;
import de.rwth.montisim.simulation.eesimulator.bus.can.CANProperties;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.events.ExecuteEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent.MessageReceiveEventData;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent.MessageSendEventData;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestCompProperties;
import de.rwth.montisim.simulation.simulator.communication.*;
import de.rwth.montisim.simulation.simulator.randomization.BasicDrivingStrategyProperties;
import de.rwth.montisim.simulation.simulator.randomization.EmptyStrategyProperties;
import de.rwth.montisim.simulation.simulator.randomization.IntersectionStrategyProperties;
import de.rwth.montisim.simulation.simulator.randomization.PlatooningStrategyProperties;
import de.rwth.montisim.simulation.vehicle.navigation.NavigationProperties;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.task.metric.MetricGoalProperties;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoalProperties;
import de.rwth.montisim.simulation.commons.boundingbox.*;
import de.rwth.montisim.simulation.commons.*;

public class TypedSimulation {
    public static void registerTypedSimulation() {
        TypedCommons.registerTypedCommons();
        Json.registerType(JavaAutopilotProperties.class);
        Json.registerType(TestAutopilotProperties.class);
        Json.registerType(RLAutopilotProperties.class);
        Json.registerType(SCGProperties.class);
        Json.registerType(ActuatorProperties.class);
        Json.registerType(BridgeProperties.class);
        Json.registerType(CANProperties.class);
        Json.registerType(ConstantBusProperties.class);
        Json.registerType(ExecuteEvent.class);
        Json.registerType(MessageReceiveEventData.class);
        Json.registerType(MessageSendEventData.class);
        Json.registerType(SensorProperties.class);
        Json.registerType(TestCompProperties.class);
        Json.registerType(NavigationProperties.class);
        Json.registerType(LidarProperties.class);
        Json.registerType(SpeedLimitServiceProperties.class);
        Json.registerType(RigidbodyPhysicsProperties.class);
        Json.registerType(ElectricalPTProperties.class);
        Json.registerType(MetricGoalProperties.class);
        Json.registerType(PathGoalProperties.class);
        Json.registerType(SimpleNetworkProperties.class);
        Json.registerType(AABB.class);
        Json.registerType(OBB.class);
        Json.registerType(DynamicObject.class);
        Json.registerType(StaticObject.class);
        Json.registerType(EmptyStrategyProperties.class);
        Json.registerType(IntersectionStrategyProperties.class);
        Json.registerType(PlatooningStrategyProperties.class);
        Json.registerType(BasicDrivingStrategyProperties.class);
        Json.registerType(DefaultPreprocessorProperties.class);
        Json.registerType(ProximityFilterProperties.class);
        Json.registerType(SequencePreprocessorProperties.class);
    }
}
