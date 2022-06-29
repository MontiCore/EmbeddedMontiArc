package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

/**
 * Reward Function that evaluates how well a vehicle follows its trajectory.
 */
public class TrajectoryRewardFunction extends RewardFunction {

  private final float TRAJECTORY_REWARD;

  private final float distance_max;

  /**
   * Initializes the Trajectory Reward Function.
   *
   * @param vehicles          Vehicle[] containing additional data about each active vehicle.
   * @param trajectory_reward Scaled reward.
   * @param distance_max      Maximum allowed distance to the next trajectory point.
   */
  public TrajectoryRewardFunction(Vehicle[] vehicles, float trajectory_reward, float distance_max) {
    super(vehicles);
    this.TRAJECTORY_REWARD = trajectory_reward;
    this.distance_max = distance_max;
  }

  @Override
  public float getRewardForVehicle(int vehicle_index) {
    Vec2 vehicle_position = this.positions[vehicle_index];
    Vec2[] vehicle_trajectory = this.navigations[vehicle_index].getCurrentTraj();

    double distanceToSeg;
    double distance = Double.MAX_VALUE;
    for (int i = 0; i < vehicle_trajectory.length - 1; i++) {
      SegmentPos currentSegment = new SegmentPos();
      currentSegment.initFromTrajectory(vehicle_position, vehicle_trajectory[i], vehicle_trajectory[i + 1]);
      if (currentSegment.projPos < 0) { //vehicle is in front of segment
        distanceToSeg = currentSegment.relPos.magnitude();
      }
      else if (currentSegment.projDistToEnd < 0) { //vehicle is behind segment
        distanceToSeg = currentSegment.endDis.magnitude();
      }
      else {
        distanceToSeg = currentSegment.dist;
      }
      if (distanceToSeg < distance) {
        distance = distanceToSeg;
      }
    }

    return -(this.TRAJECTORY_REWARD / this.distance_max) * (float) distance + this.TRAJECTORY_REWARD;

  }

  // SegmentPos taken from JavaAutopilot class for distance calculation
  private static class SegmentPos {
    Vec2 posStart = new Vec2();
    Vec2 posEnd = new Vec2();
    Vec2 dir = new Vec2();
    Vec2 normal = new Vec2();
    Vec2 relPos = new Vec2();
    Vec2 endDis = new Vec2();
    Vec2 currentPosition;

    double length;
    double projPos;
    double orthogonalPos;
    double dist;
    double projDistToEnd;

    void initFromTrajectory(Vec2 currentPosition, Vec2 posStart, Vec2 posEnd) {
      this.posStart = posStart;
      this.posEnd = posEnd;
      this.currentPosition = currentPosition;
      init();
    }

    void init() {
      IPM.subtractTo(dir, posEnd, posStart);
      length = dir.magnitude();
      if (length > 0.001) {
        IPM.multiply(dir, 1 / length);
      }
      else {
        dir.set(Double.NaN, Double.NaN);
      }
      normal.set(-dir.y, dir.x);

      IPM.subtractTo(relPos, currentPosition, posStart);
      IPM.subtractTo(endDis, currentPosition, posEnd);
      projPos = IPM.dot(dir, relPos);
      orthogonalPos = IPM.dot(normal, relPos);
      dist = Math.abs(orthogonalPos);
      projDistToEnd = length - projPos;
    }
  }

}
