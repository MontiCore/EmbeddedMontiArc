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

import java.time.Duration;
import java.time.Instant;
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
    public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime, Duration timeDiffms) {
        if(simulationObject instanceof PhysicalVehicle) {
            PhysicalVehicle physicalVehicle = (PhysicalVehicle) simulationObject;
            Plotter2D.plotOne(physicalVehicle, counter, timeDiffms, name);
        }
        counter++;
    }

    @Override
    public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, Instant totalTime) {}

    @Override
    public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime, Duration deltaTime) {}

    @Override
    public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime, Duration deltaTime) {}

    @Override
    public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime, Duration deltaTime) {}

    @Override
    public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {}
}
