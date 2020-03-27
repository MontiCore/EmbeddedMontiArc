/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator;

public abstract class EEEventProcessor {
	protected final EESimulator simulator;
    public final String name;
    public final int id;
    
    public EEEventProcessor(EESimulator simulator, String name) {
        this.simulator = simulator;
        this.name = name;
        this.id = simulator.getComponentManager().registerComponent(this);
    }

    public EEEventProcessor(EESimulator simulator, String name, int priority) {
        this.simulator = simulator;
        this.name = name;
        this.id = simulator.getComponentManager().registerComponent(this, priority);
    }


	public abstract EEComponentType getComponentType();
    public abstract void process(EEDiscreteEvent event);

    @Override
    public String toString(){
        return "EEEventProcessor \""+name+'"';
    }
}