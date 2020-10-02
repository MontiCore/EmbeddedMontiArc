package de.rwth.montisim.simulation.vehicle.task;

import de.rwth.montisim.commons.utils.Comparator;
import de.rwth.montisim.commons.utils.LTLOperator;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.vehicle.task.goal.*;
import org.junit.Test;

import static org.junit.Assert.*;

public class TaskTest {

    @Test
    public void jsonTests() throws SerializationException {
        Task origTask = new Task();

        origTask.addGoal(
                MetricGoal.newBuilder()
                .setProperty(VehicleProperty.SPEED)
                .never()
                .greater(100, "m/s")
                .build()
        );
        origTask.addGoal(PathGoal.newBuilder()
                .eventually()
                .arrive(0, 0)
                .arrive(1, 0)
                .arrive(1, 1)
                .withInRange(10)
                .build()
        );

        // check if deserialization reproduce the same object
        String json = Json.toJson(origTask);
        Task deserialized = Json.instantiateFromJson(json, Task.class);
        assertEquals(origTask.getGoals().size(), deserialized.getGoals().size());
        for (int i = 0; i < origTask.getGoals().size(); i++) {
            assertEquals(deserialized.getGoals().get(i), origTask.getGoals().get(i));
        }
    }

    @Test
    public void metricBuilderTests() {
        Goal goal;

        // test LTL operators
        goal = MetricGoal.newBuilder().never().build();
        assertEquals(LTLOperator.NEVER, goal.getLtlOperator());
        goal = MetricGoal.newBuilder().always().build();
        assertEquals(LTLOperator.ALWAYS, goal.getLtlOperator());
        goal = MetricGoal.newBuilder().eventually().build();
        assertEquals(LTLOperator.EVENTUALLY, goal.getLtlOperator());
        goal = MetricGoal.newBuilder().until().build();
        assertEquals(LTLOperator.UNTIL, goal.getLtlOperator());

        // test vehicle properties
        goal = MetricGoal.newBuilder().setProperty(VehicleProperty.ACCELERATION).build();
        assertEquals(VehicleProperty.ACCELERATION, ((MetricGoal) goal).getProperty());
        goal = MetricGoal.newBuilder().setProperty(VehicleProperty.SPEED).build();
        assertEquals(VehicleProperty.SPEED, ((MetricGoal) goal).getProperty());
        goal = MetricGoal.newBuilder().setProperty(VehicleProperty.MIN_BATTERY).build();
        assertEquals(VehicleProperty.MIN_BATTERY, ((MetricGoal) goal).getProperty());

        // test comparators, target values and target units
        goal = MetricGoal.newBuilder().less(1, "m/s").build();
        assertEquals(Comparator.LESS, ((MetricGoal) goal).getComparator());
        assert 1d ==  ((MetricGoal) goal).getValue();
        assertEquals("m/s", ((MetricGoal) goal).getUnit());
        goal = MetricGoal.newBuilder().lessEqual(1, "m/s").build();
        assertEquals(Comparator.LESS_EQUAL, ((MetricGoal) goal).getComparator());
        goal = MetricGoal.newBuilder().greater(1, "m/s").build();
        assertEquals(Comparator.GREATER, ((MetricGoal) goal).getComparator());
        goal = MetricGoal.newBuilder().greaterEqual(1, "m/s").build();
        assertEquals(Comparator.GREATER_EQUAL, ((MetricGoal) goal).getComparator());
        goal = MetricGoal.newBuilder().equals(1, "m/s").build();
        assertEquals(Comparator.EQUAL, ((MetricGoal) goal).getComparator());
    }
}