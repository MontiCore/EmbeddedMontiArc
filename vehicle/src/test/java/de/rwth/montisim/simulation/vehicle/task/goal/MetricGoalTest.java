package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.commons.physicalvalue.PhysicalValueRegistry;
import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;
import junit.framework.TestCase;

import java.util.Optional;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class MetricGoalTest extends TestCase {

    public void testNever() {
        Goal goal = MetricGoal.newBuilder()
                .setProperty(TrueVelocity.VALUE_NAME)
                .never()
                .greater(100, "m/s")
                .build();

        Vehicle mockVehicle = mock(Vehicle.class);
        PhysicalValueRegistry mockPValue = mock(PhysicalValueRegistry.class);
        TrueVelocity mockVelocity = mock(TrueVelocity.class);
        when(mockVehicle.getPhysicalValues()).thenReturn(mockPValue);
        when(mockPValue.getPhysicalValue(TrueVelocity.VALUE_NAME)).thenReturn(mockVelocity);

        // speed ok
        when(mockVelocity.get()).thenReturn(0d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.SUCCEEDED, goal.getStatus());

        // speed limit exceeded
        when(mockVelocity.get()).thenReturn(200d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.FAILED, goal.getStatus());

        // should always be FAILED even if vehicle slowed down
        when(mockVelocity.get()).thenReturn(0d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.FAILED, goal.getStatus());
    }

    public void testAlways() {
        Goal goal = MetricGoal.newBuilder()
                .setProperty(TrueVelocity.VALUE_NAME)
                .always()
                .less(100, "m/s")
                .build();

        Vehicle mockVehicle = mock(Vehicle.class);
        PhysicalValueRegistry mockPValue = mock(PhysicalValueRegistry.class);
        TrueVelocity mockVelocity = mock(TrueVelocity.class);
        when(mockVehicle.getPhysicalValues()).thenReturn(mockPValue);
        when(mockPValue.getPhysicalValue(TrueVelocity.VALUE_NAME)).thenReturn(mockVelocity);

        // speed ok
        when(mockVelocity.get()).thenReturn(0d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.SUCCEEDED, goal.getStatus());

        // speed limit exceeded
        when(mockVelocity.get()).thenReturn(200d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.FAILED, goal.getStatus());

        // should always be FAILED even if vehicle slowed down
        when(mockVelocity.get()).thenReturn(0d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.FAILED, goal.getStatus());
    }

}