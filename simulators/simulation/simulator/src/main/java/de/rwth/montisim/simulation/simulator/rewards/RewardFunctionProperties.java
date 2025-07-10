package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

import java.time.Duration;

/**
 * Abstract class that defines the parameters of every reward function and a function that builds this certain reward function.
 */
public abstract class RewardFunctionProperties {

  public abstract RewardFunction build(Vehicle[] vehicles, Duration tickDuration);

}
