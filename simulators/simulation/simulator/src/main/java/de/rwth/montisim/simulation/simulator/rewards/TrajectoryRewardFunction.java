package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;
import java.util.Optional;

/**
 * Reward Function that evaluates how well a vehicle follows its trajectory.
 */
public class TrajectoryRewardFunction extends RewardFunction {

  private final float TRAJECTORY_FOLLOWING_REWARD;
  private final float TOTAL_PATH_PROGRESS_REWARD_SCALING;
  private final float PATH_PROGRESS_DERIVATIVE_REWARD_SCALING;

  private final float distance_max;

  private final double[] total_path_distance;

  private final double[] old_remaining_path_distance_score;

  /**
   * Initializes the Trajectory Reward Function.
   *
   * @param vehicles          Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration      Duration between two Updates of the Simulator.
   * @param TRAJECTORY_FOLLOWING_REWARD Total reward given to a vehicle exactly following the trajectory.
   * @param TOTAL_PATH_PROGRESS_REWARD_SCALING Scaled reward given to the total progress on the path.
   * @param PATH_PROGRESS_DERIVATIVE_REWARD_SCALING Scaled reward given to the change in progress on the path.
   * @param distance_max      Maximum allowed distance to the next trajectory point.
   */
  public TrajectoryRewardFunction(Vehicle[] vehicles, Duration tickDuration, float TRAJECTORY_FOLLOWING_REWARD, float TOTAL_PATH_PROGRESS_REWARD_SCALING, float PATH_PROGRESS_DERIVATIVE_REWARD_SCALING, float distance_max) {
    super(vehicles, tickDuration);
    this.TRAJECTORY_FOLLOWING_REWARD = TRAJECTORY_FOLLOWING_REWARD;
    this.TOTAL_PATH_PROGRESS_REWARD_SCALING = TOTAL_PATH_PROGRESS_REWARD_SCALING;
    this.PATH_PROGRESS_DERIVATIVE_REWARD_SCALING = PATH_PROGRESS_DERIVATIVE_REWARD_SCALING;
    this.distance_max = distance_max;
    this.total_path_distance = new double[this.NUMBER_OF_VEHICLES];
    for (int i = 0; i < this.NUMBER_OF_VEHICLES; i++) {
      Optional<Double> total_dist = this.navigations[i].getRemainingPathLength();
      if (!total_dist.isPresent()) {
        new Exception("Something went horribly wrong!").printStackTrace();
      }
      this.total_path_distance[i] = total_dist.get(); // if not present something went horribly wrong
    }
    this.old_remaining_path_distance_score = new double[this.NUMBER_OF_VEHICLES];
    for (int i = 0; i < this.NUMBER_OF_VEHICLES; i++) {
      this.old_remaining_path_distance_score[i] = 0;
    }
  }

  @Override
  public float getRewardForVehicle(int vehicle_index, int step) {
    Vec2 vehicle_position = (Vec2) this.positions[vehicle_index].get();
    Vec2[] vehicle_trajectory = this.navigations[vehicle_index].getCurrentTraj();

    float reward = 0;

    // Stay on the Path
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
    reward += -(this.TRAJECTORY_FOLLOWING_REWARD / this.distance_max) * (float) distance + this.TRAJECTORY_FOLLOWING_REWARD;

    // Progress on the Path
    Optional<Double> remaining_length = this.navigations[vehicle_index].getRemainingPathLength();
    double old_score = old_remaining_path_distance_score[vehicle_index];
    double new_score = old_score;
    if (remaining_length.isPresent()) { // So, apparently, sometimes, some data in Navigation just isn't there. Just reward the old score...
      new_score = (1 - (remaining_length.get() / total_path_distance[vehicle_index]));
      old_remaining_path_distance_score[vehicle_index] = new_score;
    }
    // reward the total progress
    reward += this.TOTAL_PATH_PROGRESS_REWARD_SCALING * old_remaining_path_distance_score[vehicle_index];
    // reward the change in progress
    reward += this.PATH_PROGRESS_DERIVATIVE_REWARD_SCALING * (new_score - old_score);

    return reward;
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
