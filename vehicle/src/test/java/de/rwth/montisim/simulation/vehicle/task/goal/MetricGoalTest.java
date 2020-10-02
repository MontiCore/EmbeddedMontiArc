package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import junit.framework.TestCase;

import static org.mockito.Mockito.mock;

public class MetricGoalTest extends TestCase {

    public void testNever() {
        Goal goal = MetricGoal.newBuilder()
                .setProperty(VehicleProperty.SPEED)
                .never()
                .greater(100, "m/s")
                .build();

        Vehicle v = mock(Vehicle.class);
        v.physicalObject = mock(DynamicObject.class);

        // speed ok
        v.physicalObject.velocity = new Vec3(0, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.SUCCEEDED, goal.getStatus());

        // speed limit exceeded
        v.physicalObject.velocity = new Vec3(200, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.FAILED, goal.getStatus());

        // should always be FAILED even if vehicle slowed down
        v.physicalObject.velocity = new Vec3(0, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.FAILED, goal.getStatus());
    }

    public void testAlways() {
        Goal goal = MetricGoal.newBuilder()
                .setProperty(VehicleProperty.SPEED)
                .always()
                .less(100, "m/s")
                .build();

        Vehicle v = mock(Vehicle.class);
        v.physicalObject = mock(DynamicObject.class);

        // speed ok
        v.physicalObject.velocity = new Vec3(0, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.SUCCEEDED, goal.getStatus());

        // speed limit exceeded
        v.physicalObject.velocity = new Vec3(200, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.FAILED, goal.getStatus());

        // should always be FAILED even if vehicle slowed down
        v.physicalObject.velocity = new Vec3(0, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.FAILED, goal.getStatus());
    }

}