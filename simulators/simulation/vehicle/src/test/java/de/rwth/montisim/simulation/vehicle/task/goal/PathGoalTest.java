/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.task.goal;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import java.util.Optional;

import de.rwth.montisim.simulation.commons.DynamicObject;
import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;
import de.rwth.montisim.simulation.vehicle.task.Goal;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoal;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoalProperties;

import org.junit.Test;


public class PathGoalTest {
    @Test
    public void testEventually() {
        PathGoal goal = (PathGoal) new PathGoalProperties()
                .reach(100, 0)
                .reach(200, 0)
                .reach(300, 0)
                .withinRange(10)
                .eventually()
                .build(null, null, null);

        Vehicle v = mock(Vehicle.class);
        v.physicalObject = mock(DynamicObject.class);
        Navigation nav = mock(Navigation.class);
        Optional<Navigation> navv = Optional.of(nav);

        v.physicalObject.pos = new Vec3(0, 0, 0);
        goal.updateDriveTarget(v, navv);
        assertEquals(TaskStatus.RUNNING, goal.getStatus());

        v.physicalObject.pos = new Vec3(95, 0, 0);
        goal.updateDriveTarget(v, navv);
        assertEquals(TaskStatus.RUNNING, goal.getStatus());

        v.physicalObject.pos = new Vec3(195, 0, 0);
        goal.updateDriveTarget(v, navv);
        assertEquals(TaskStatus.RUNNING, goal.getStatus());

        v.physicalObject.pos = new Vec3(295, 0, 0);
        goal.updateDriveTarget(v, navv);
        assertEquals(TaskStatus.SUCCEEDED, goal.getStatus());
    }

    @Test
    public void testNever() {
        Goal goal = new PathGoalProperties()
                .reach(100, 0)
                .reach(200, 0)
                .reach(300, 0)
                .withinRange(10)
                .never()
                .build(null, null, null);

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
