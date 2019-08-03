package simulation.EESimulator;

import java.util.UUID;

import org.apache.commons.math3.exception.NullArgumentException;

abstract class AbstractEEComponent implements EEComponent{

	private final EESimulator simulator;
	
	private final EEComponentType componentType;
	
	private final UUID Id;
	
	public AbstractEEComponent(EESimulator simulator, EEComponentType type) {
		if (simulator == null || type == null) {
			throw new NullArgumentException();
		}
		this.simulator = simulator;
		this.componentType = type;
		this.Id = UUID.randomUUID();
	}
	
	@Override
	public EEComponentType getComponentType() {
		return componentType;
	}
	
	@Override
	public EESimulator getSimulator() {
		return simulator;
	}

	@Override
	public UUID getId() {
		return Id;
	}
}
