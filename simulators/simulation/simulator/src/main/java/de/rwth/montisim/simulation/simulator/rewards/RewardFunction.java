package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValue;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

import java.time.Duration;
import java.util.Arrays;

/**
 * Abstract class that specifies a reward function, evaluating an action taken by an agent in the environment.
 */
public abstract class RewardFunction {

  final int NUMBER_OF_VEHICLES;
  final Vehicle[] vehicles;
  final Navigation[] navigations;
  final PhysicalValue[] positions;
  final PhysicalValue[] velocities;
  final PhysicalValue[] angles;
  final Duration tickDuration;

  /**
   * Default constructor that initializes the parameters required for any given reward function: Navigation and Data of each Vehicle.
   *
   * @param vehicles     Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration Duration between two Updates of the Simulator.
   */
  public RewardFunction(Vehicle[] vehicles, Duration tickDuration) {
    this.NUMBER_OF_VEHICLES = vehicles.length;
    this.vehicles = vehicles;
    this.navigations = Arrays.stream(vehicles).map(vehicle -> (Navigation) vehicle.eesystem.getComponent("Navigation").get()).toArray(Navigation[]::new);
    this.positions = Arrays.stream(vehicles).map(vehicle -> vehicle.physicalValues.getPhysicalValue("true_position")).toArray(PhysicalValue[]::new);
    this.velocities = Arrays.stream(vehicles).map(vehicle -> vehicle.physicalValues.getPhysicalValue("true_velocity")).toArray(PhysicalValue[]::new);
    this.angles = Arrays.stream(vehicles).map(vehicle -> vehicle.physicalValues.getPhysicalValue("true_compass")).toArray(PhysicalValue[]::new);
    this.tickDuration = tickDuration;
  }

  /**
   * Evaluates the current environment as a whole and calculates the reward score for it.
   * The default implementation returns the cumulated reward of each vehicle {@link #getRewardForVehicle(int, int)}.
   *
   * @param step Current step iteration.
   *
   * @return the reward for the current environment.
   */
  public float getReward(int step) {
    float reward = 0;
    for (int i = 0; i < NUMBER_OF_VEHICLES; i++) {
      reward += getRewardForVehicle(i, step);
    }
    return reward;
  }

  /**
   * Evaluates the current environment from the perspective of a single vehicle and calculates the reward score for it.
   *
   * @param vehicle_index Index of the vehicle to calculate the reward for.
   * @param step Current step iteration.
   * @return the reward for the current environment from the specified vehicle's perspective.
   */
  public abstract float getRewardForVehicle(int vehicle_index, int step);

}
