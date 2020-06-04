/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator;

import java.time.Instant;

import de.rwth.montisim.commons.simulation.TimeUpdate;

public class SimulationLoop {
    final Simulator simulator;
    final SimulationConfig config;
    Instant simulationTime;
    public SimulationLoop(Simulator simulator, SimulationConfig config){
        this.simulator = simulator;
        this.config = config;
        this.simulationTime = config.simulationStart;
    }

    public boolean run(){
        try {
            while (!simulator.finished()){
                TimeUpdate tu = new TimeUpdate(simulationTime, config.tickDuration);
                simulator.update(tu);
                simulationTime = tu.newTime;
            }
        } catch(Exception e){
            e.printStackTrace();
            return false;
        }
        return true;
    }
}