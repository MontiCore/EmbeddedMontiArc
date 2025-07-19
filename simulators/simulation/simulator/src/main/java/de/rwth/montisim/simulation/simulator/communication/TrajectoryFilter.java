package de.rwth.montisim.simulation.simulator.communication;

import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.commons.utils.Vec2;

import java.util.Comparator;
import java.util.Optional;
import java.util.Vector;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 * Preprocessor that prioritizes vehicles, that might collide in the near future.
 */
public class TrajectoryFilter implements Preprocessor {

  private final int maxNumberOfVehicles;
  private final boolean addIndicator;

  private final float VEHICLE_WIDTH;
  private final float VEHICLE_LENGTH;
  private final float VEHICLE_RADIUS;
  private final int MAX_STATES_PER_VEHICLE;

  private final float timeStepTop; // performance optimization

  /**
   * Constructor of the TrajectoryFilter
   *
   * @param VEHICLE_WIDTH          Width of a vehicle
   * @param VEHICLE_LENGTH         Height of a vehicle
   * @param MAX_STATES_PER_VEHICLE specifies how many future states should be calculated for a vehicle
   */
  public TrajectoryFilter(float VEHICLE_WIDTH, float VEHICLE_LENGTH, int MAX_STATES_PER_VEHICLE, int maxNumberOfVehicles, boolean addIndicator) {
    this.maxNumberOfVehicles = maxNumberOfVehicles;
    this.addIndicator = addIndicator;
    this.VEHICLE_WIDTH = VEHICLE_WIDTH;
    this.VEHICLE_LENGTH = VEHICLE_LENGTH;
    this.MAX_STATES_PER_VEHICLE = MAX_STATES_PER_VEHICLE;
    this.VEHICLE_RADIUS = (float) Math.sqrt(Math.pow(VEHICLE_WIDTH, 2) + Math.pow(VEHICLE_LENGTH, 2)) / 2;
    this.timeStepTop = (float) Math.sqrt(2) * (2 * VEHICLE_RADIUS);
  }

  /**
   * Computes the coarse time step used to calculate the future positions of a vehicle.
   *
   * @param velocity_host   Velocity of the first vehicle
   * @param velocity_remote Velocity of the second vehicle
   * @return the coarse time step
   */
  private float computeTimeStep(float velocity_host, float velocity_remote) {
    return this.timeStepTop / Math.max(Math.max(Math.abs(velocity_host), Math.abs(velocity_remote)), 0.01f);
  }

  /**
   * Calculates the future states for two vehicles
   *
   * @param hostState   The current autopilot state of the first vehicle
   * @param remoteState The current autopilot state of the second vehicle
   * @return a pair of future states
   */
  private Pair<Vector<State>, Vector<State>> getFutureStates(float[] hostState, float[] remoteState) {
    // get location, velocity, and trajectory of host
    float host_x = hostState[21];
    float host_y = hostState[22];
    float host_velocity = hostState[24];
    Vector<Vec2> host_trajectory = new Vector<>();
    for (int index = 0; index < hostState[20]; index++) {
      host_trajectory.add(new Vec2(hostState[index], hostState[index + 10]));
    }

    // get location, velocity, and trajectory of remote
    float remote_x = remoteState[21];
    float remote_y = remoteState[22];
    float remote_velocity = remoteState[24];
    Vector<Vec2> remote_trajectory = new Vector<>();
    for (int index = 0; index < remoteState[20]; index++) {
      remote_trajectory.add(new Vec2(remoteState[index], remoteState[index + 10]));
    }

    // create future states
    float timeStep = computeTimeStep(host_velocity, remote_velocity);

    Vector<State> hostFutureStates = new Vector<>();
    hostFutureStates.add(new State(host_x, host_y, host_velocity));
    Vector<State> remoteFutureStates = new Vector<>();
    remoteFutureStates.add(new State(remote_x, remote_y, remote_velocity));
    for (int i = 0; i < MAX_STATES_PER_VEHICLE; i++) {
      Optional<State> next = hostFutureStates.lastElement().calculateNextState(new Vec2(host_x, host_y), host_trajectory, timeStep);
      if (next.isPresent()) {
        hostFutureStates.add(next.get());
      }
      else {
        break;
      }
    }
    for (int i = 0; i < MAX_STATES_PER_VEHICLE; i++) {
      Optional<State> next = remoteFutureStates.lastElement().calculateNextState(new Vec2(remote_x, remote_y), remote_trajectory, timeStep);
      if (next.isPresent()) {
        remoteFutureStates.add(next.get());
      }
      else {
        break;
      }
    }

    return new Pair<>(hostFutureStates, remoteFutureStates);
  }

  /**
   * Scores the future states of two vehicles depending on possible collisions
   * To be used after {@link TrajectoryFilter#getFutureStates(float[], float[])}
   *
   * @param futureStatePair The pair of future states of both of the vehicles
   * @return A high score if a collision will occur soon, else a low score (0)
   */
  private float scoreFutureStates(Pair<Vector<State>, Vector<State>> futureStatePair) {
    Vector<State> hostFutureStates = futureStatePair.getKey();
    Vector<State> remoteFutureStates = futureStatePair.getValue();

    float score = 0;

    // loop over both lists and score all combinations of states
    for (int host_index = 0; host_index < hostFutureStates.size(); host_index++) {
      State hostState = hostFutureStates.get(host_index);
      for (int remote_index = 0; remote_index < remoteFutureStates.size(); remote_index++) {
        State remoteState = remoteFutureStates.get(remote_index);
        if (hostState.intersectsOtherState(remoteState)) {
          // score is higher when close to the current positions
          // score is lower when different distant in time apart
          score += MAX_STATES_PER_VEHICLE / (float) (Math.min(host_index, remote_index) + 1) + 1 / (float) (Math.abs(host_index - remote_index) + 1);
        }
      }
    }
    return score;
  }

  @Override
  public float[] preprocessState(float[] vehicleState, float[][] otherStates, int statePacketLength) {
    /*
      For each other vehicle, do the following:
       - Calculate the future states
       - Score the future states
      Then, sort the other vehicles by their scores in descending order.
     */
    Comparator<Pair<Integer, Float>> c = Comparator.comparing(Pair::getValue);
    Vector<Pair<Integer, Float>> scores = IntStream.range(0, otherStates.length).mapToObj(index -> new Pair<>(index, scoreFutureStates(getFutureStates(vehicleState, otherStates[index])))).sorted(c.reversed()).collect(Collectors.toCollection(Vector::new));

    float[] preprocessedState = new float[vehicleState.length + this.maxNumberOfVehicles * (statePacketLength + (this.addIndicator ? 1 : 0))];

    System.arraycopy(vehicleState, 0, preprocessedState, 0, vehicleState.length);
    for (int index = 0; index < Math.min(scores.size(), maxNumberOfVehicles); index++) {
      System.arraycopy(otherStates[scores.get(index).getKey()], 0, preprocessedState, vehicleState.length + index * (statePacketLength + (this.addIndicator ? 1 : 0)), statePacketLength);
      if(this.addIndicator) {
        preprocessedState[vehicleState.length + index * (statePacketLength + 1) + statePacketLength] = 1;
      }
    }

    return preprocessedState;
  }

  @Override
  public int getStateLength(int inputStateLength, int inputStatePacketLength) {
    return inputStateLength;
  }

  @Override
  public int getStatePacketLength(int inputStateLength, int inputStatePacketLength) {
    if (addIndicator) {
      return inputStatePacketLength + 1;
    } else {
      return inputStatePacketLength;
    }
  }

  /**
   * State class to wrap the current state values into an object.
   */
  class State {
    float x;
    float y;
    float velocity;
    float distance_from_start;

    private State(float x, float y, float velocity, float distance_from_start) {
      this.x = x;
      this.y = y;
      this.velocity = velocity;
      this.distance_from_start = distance_from_start;
    }

    public State(float x, float y, float velocity) {
      this(x, y, velocity, 0);
    }

    public State(Vec2 start, float velocity) {
      this((float) start.x, (float) start.y, velocity, 0);
    }

    /**
     * Calculates the next state depending on the current state, vehicles trajectory, and distance in time (timeStep)
     *
     * @param start      The actual current position of the vehicle. Not to be confused with the current {@link State}.
     * @param trajectory The current trajectory of the vehicle. The next state will be strictly on this trajectory.
     * @param timeStep   If the vehicle drove at the constant velocity, where would it be after this timeStep?
     * @return Only returns a state, if the next state would still be on the trajectory. Otherwise, returns {@link Optional#empty()}.
     */
    public Optional<State> calculateNextState(Vec2 start, Vector<Vec2> trajectory, float timeStep) {
      float new_distance_from_start = this.distance_from_start + this.velocity * timeStep; // this must be the new distance from the start

      // check if the trajectory is long enough
      Vec2 lastPoint = start;
      Vec2 currentPoint = null;
      float trajectoryDistance = 0;
      float currentSegmentDistance = 0;
      boolean foundSegment = false;
      for (Vec2 vec2 : trajectory) {
        currentPoint = vec2;
        currentSegmentDistance = (float) lastPoint.distance(currentPoint);
        if (trajectoryDistance + currentSegmentDistance > new_distance_from_start) {
          foundSegment = true;
          break;
        }
        else {
          trajectoryDistance += currentSegmentDistance;
          lastPoint = currentPoint;
        }
      }
      if (!foundSegment)
        return Optional.empty();

      assert (new_distance_from_start >= trajectoryDistance);
      float dist = new_distance_from_start - trajectoryDistance;

      // linearly interpolate the next states location
      float new_x = (float) (lastPoint.x + (dist / currentSegmentDistance) * (currentPoint.x - lastPoint.x));
      float new_y = (float) (lastPoint.y + (dist / currentSegmentDistance) * (currentPoint.y - lastPoint.y));

      return Optional.of(new State(new_x, new_y, this.velocity, new_distance_from_start));
    }

    /**
     * Calculates the distance between two states locations. Does not take difference in time into consideration.
     *
     * @param other the other state
     * @return the distance
     */
    public float distanceToState(State other) {
      return (float) new Vec2(this.x, this.y).distance(other.x, other.y);
    }

    /**
     * Whether one state intersects another state. Does not take difference in time into consideration.
     *
     * @param other the other state
     * @return whether the states intersect one and another
     */
    public boolean intersectsOtherState(State other) {
      return distanceToState(other) <= 2 * VEHICLE_RADIUS;
    }

    @Override
    public String toString() {
      return "[" + this.x + ", " + this.y + ", " + this.velocity + ", " + this.distance_from_start + "]";
    }
  }
}
