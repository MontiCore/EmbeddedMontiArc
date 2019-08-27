/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.simulator;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopExecutable;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopNotifiable;
import simulation.util.Plotter2D;
import simulation.vehicle.PhysicalVehicle;
import java.util.List;

/**
 * Debug Plotter that visualizes the movement for a simulation with only one physical vehicle
 */
public class SimulationDebugPlotter extends SimulationLoopNotifiable {

    private long counter = 0;
    private String name;

    public SimulationDebugPlotter(String name){
        this.name = name;
    }

    @Override
    public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long timeDiffms) {
        if(simulationObject instanceof PhysicalVehicle) {
            PhysicalVehicle physicalVehicle = (PhysicalVehicle) simulationObject;
            Plotter2D.plotOne(physicalVehicle, counter, timeDiffms, name);
        }
        counter++;
    }

    @Override
    public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, long totalTime) {}

    @Override
    public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {}

    @Override
    public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {}

    @Override
    public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {}

    @Override
    public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {}
}
