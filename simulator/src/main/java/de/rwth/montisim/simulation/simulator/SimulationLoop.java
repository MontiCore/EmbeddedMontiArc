/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator;

import java.time.Instant;

import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.commons.simulation.TimeUpdate;

public class SimulationLoop {
    final Simulator simulator;
    final SimulationConfig config;
    
    Instant simulationTime;
    public SimulationLoop(Simulator simulator, SimulationConfig config){
        this.simulator = simulator;
        this.config = config;
        this.simulationTime = config.start_time;
    }

    public TaskStatus run() {
        try {
            do {
                TaskStatus res = simulator.status();
                if (res != TaskStatus.RUNNING){

                    return res;
                }
                TimeUpdate tu = new TimeUpdate(simulationTime, config.tick_duration);
                simulator.update(tu);
                simulationTime = tu.newTime;

            } while(true);
        } catch(Exception e){
            e.printStackTrace();
            return TaskStatus.FAILED;
        }
    }
}