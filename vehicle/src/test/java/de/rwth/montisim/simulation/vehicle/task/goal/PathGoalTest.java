package de.rwth.montisim.simulation.vehicle.task.goal;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class PathGoalTest {
    @Test
    public void testEventually() {
        Goal goal = PathGoal.newBuilder()
                .eventually()
                .arrive(new Vec2(100, 0))
                .arrive(new Vec2(200, 0))
                .arrive(new Vec2(300, 0))
                .withInRange(10)
                .build();

        Vehicle v = mock(Vehicle.class);
        v.physicalObject = mock(DynamicObject.class);

        v.physicalObject.pos = new Vec3(0, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.RUNNING, goal.getStatus());

        v.physicalObject.pos = new Vec3(95, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.RUNNING, goal.getStatus());

        v.physicalObject.pos = new Vec3(195, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.RUNNING, goal.getStatus());

        v.physicalObject.pos = new Vec3(295, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.SUCCEEDED, goal.getStatus());
    }

    @Test
    public void testNever() {
        Goal goal = PathGoal.newBuilder()
                .never()
                .arrive(new Vec2(100, 0))
                .arrive(new Vec2(200, 0))
                .arrive(new Vec2(300, 0))
                .withInRange(10)
                .build();

        Vehicle v = mock(Vehicle.class);
        v.physicalObject = mock(DynamicObject.class);

        v.physicalObject.pos = new Vec3(0, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.SUCCEEDED, goal.getStatus());

        v.physicalObject.pos = new Vec3(95, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.FAILED, goal.getStatus());

        v.physicalObject.pos = new Vec3(0, 0, 0);
        goal.update(v);
        assertEquals(TaskStatus.FAILED, goal.getStatus());
    }
}