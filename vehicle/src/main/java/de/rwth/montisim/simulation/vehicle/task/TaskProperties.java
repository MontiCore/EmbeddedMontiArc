package de.rwth.montisim.simulation.vehicle.task;

import java.util.Optional;
import java.util.Vector;

import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;
import de.rwth.montisim.simulation.vehicle.navigation.NavigationProperties;

public class TaskProperties {
    public Vector<GoalProperties> goals = new Vector<>();

    public void addGoal(GoalProperties goal) {
        goals.add(goal);
    }

    public Task build(Vehicle vehicle, OsmMap map, World world) {
        Optional<EEEventProcessor> res =vehicle.eesystem.getComponentManager().getComponent(NavigationProperties.NAME);
        if (res.isEmpty()) throw new IllegalArgumentException("The Vehicle Tasks need the Navigation component");
        EEEventProcessor r2 = res.get();
        if (!(r2 instanceof Navigation)) throw new IllegalArgumentException("Expected EEComponent with name 'Navigation' to be of type Navigation.");
        Task t = new Task(this, (Navigation)r2);
        for (GoalProperties goal : goals) {
            t.addGoal(goal.build(vehicle, map, world));
        }
        return t;
    }
}
