/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.task.goal;

import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValueRegistry;
import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.commons.utils.Comparator;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;
import de.rwth.montisim.simulation.vehicle.task.Goal;
import de.rwth.montisim.simulation.vehicle.task.metric.MetricGoalProperties;
import junit.framework.TestCase;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class MetricGoalTest extends TestCase {

    public void testNever() {
        Vehicle mockVehicle = mock(Vehicle.class);
        PhysicalValueRegistry mockPValue = mock(PhysicalValueRegistry.class);
        TrueVelocity mockVelocity = mock(TrueVelocity.class);
        when(mockVehicle.getPhysicalValues()).thenReturn(mockPValue);
        when(mockPValue.getPhysicalValue(TrueVelocity.VALUE_NAME)).thenReturn(mockVelocity);

        Goal goal = new MetricGoalProperties()
                .compare(TrueVelocity.VALUE_NAME)
                .with(100, "m/s")
                .operator(Comparator.GREATER)
                .never()
                .build(mockVehicle, null, null);


        // speed ok
        when(mockVelocity.getValue()).thenReturn(0d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.SUCCEEDED, goal.getStatus());

        // speed limit exceeded
        when(mockVelocity.getValue()).thenReturn(200d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.FAILED, goal.getStatus());

        // should always be FAILED even if vehicle slowed down
        when(mockVelocity.getValue()).thenReturn(0d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.FAILED, goal.getStatus());
    }

    public void testAlways() {
        Vehicle mockVehicle = mock(Vehicle.class);
        PhysicalValueRegistry mockPValue = mock(PhysicalValueRegistry.class);
        TrueVelocity mockVelocity = mock(TrueVelocity.class);
        when(mockVehicle.getPhysicalValues()).thenReturn(mockPValue);
        when(mockPValue.getPhysicalValue(TrueVelocity.VALUE_NAME)).thenReturn(mockVelocity);

        Goal goal = new MetricGoalProperties()
                .compare(TrueVelocity.VALUE_NAME)
                .with(100, "m/s")
                .operator(Comparator.LESS)
                .always()
                .build(mockVehicle, null, null);


        // speed ok
        when(mockVelocity.getValue()).thenReturn(0d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.SUCCEEDED, goal.getStatus());

        // speed limit exceeded
        when(mockVelocity.getValue()).thenReturn(200d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.FAILED, goal.getStatus());

        // should always be FAILED even if vehicle slowed down
        when(mockVelocity.getValue()).thenReturn(0d);
        goal.update(mockVehicle);
        assertEquals(TaskStatus.FAILED, goal.getStatus());
    }

}
