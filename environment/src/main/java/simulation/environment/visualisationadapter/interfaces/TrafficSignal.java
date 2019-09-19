/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.environment.visualisationadapter.interfaces;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopExecutable;

/**
 * Created by Shahriar Robbani on 26.01.17.
 */
@Deprecated
public interface TrafficSignal extends SimulationLoopExecutable {
    public TrafficSignalStatus getSignalA();

    public TrafficSignalStatus getSignalB();
}
