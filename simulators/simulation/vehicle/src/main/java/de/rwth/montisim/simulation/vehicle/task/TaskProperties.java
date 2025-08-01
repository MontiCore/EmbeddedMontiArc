/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.task;

import java.util.Optional;
import java.util.Vector;

import de.rwth.montisim.simulation.eesimulator.EEComponent;
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

    public Task build(Vehicle vehicle, Optional<OsmMap> map, Optional<World> world) {
        Optional<EEComponent> res = vehicle.eesystem.getComponent(NavigationProperties.NAME);
        Optional<Navigation> nav = Optional.empty();
        if (res.isPresent()) {
            EEComponent r2 = res.get();
            if (!(r2 instanceof Navigation))
                throw new IllegalArgumentException("Expected EEComponent with name 'Navigation' to be of type Navigation.");
            nav = Optional.of((Navigation) r2);
        }

        Task t = new Task(this, nav);
        for (GoalProperties goal : goals) {
            t.addGoal(goal.build(vehicle, map, world));
        }
        return t;
    }
}
