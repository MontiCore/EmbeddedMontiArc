package de.rwth.montisim.simulation.simulator;

import de.rwth.montisim.simulation.commons.physicalvalue.*;
import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.commons.*;
import de.rwth.montisim.commons.utils.BuildObject;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.task.Task;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;
import de.rwth.montisim.simulation.eecomponents.speed_limit.SpeedLimitService;
import de.rwth.montisim.simulation.vehicle.physicalvalues.*;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import de.rwth.montisim.simulation.eecomponents.autopilots.RLAutopilot;

import java.util.Optional;
import java.util.ArrayList;
import java.util.List;


// This class is responsible for the reward calculation in 
// reinforcement learning scenarios

public class RLRewardCalculator {

    private final double DEFAULT_SPEED_LIMIT = 50.0; //in km/h
    private PhysicalValue[] truePositions;
    private PhysicalValue[] trueVelocity;
    private Navigation[] navigations;
    private SpeedLimitService[] speedLimitServices;
    private double[] vehicleLengths;
    private int[] maxPathIndex;
    private double[] currentSpeedLimits; //in km/h

    private RLAutopilot[] steering;
    private RLAutopilot[] gas;
    private RLAutopilot[] brakes;
    private Vehicle[] vehicle;

    public RLRewardCalculator(Navigation[] navigations, Vehicle[] vehicles) {
        truePositions = new TruePosition[vehicles.length];
        trueVelocity = new TrueVelocity[vehicles.length];
        steering = new RLAutopilot[vehicles.length];
        gas = new RLAutopilot[vehicles.length];
        brakes = new RLAutopilot[vehicles.length];
        speedLimitServices = new SpeedLimitService[vehicles.length];
        vehicle = vehicles;

        for (int i = 0; i < truePositions.length; i++) {
            truePositions[i] = vehicles[i].physicalValues.getPhysicalValue("true_position");
            trueVelocity[i] = vehicles[i].physicalValues.getPhysicalValue("true_velocity");
            if (!vehicles[i].eesystem.getComponent("SpeedLimit").isPresent()) {
                speedLimitServices[i] = null;
            } else {
                speedLimitServices[i] = (SpeedLimitService) vehicles[i].eesystem.getComponent("SpeedLimit").get();
            }
            if (!vehicles[i].eesystem.getComponent("RLAutopilot").isPresent()) {
                steering[i] = null;
                gas[i] = null;
                brakes[i] = null;
            } else {
                steering[i] = (RLAutopilot) vehicles[i].eesystem.getComponent("RLAutopilot").get();
                gas[i] = (RLAutopilot) vehicles[i].eesystem.getComponent("RLAutopilot").get();
                brakes[i] = (RLAutopilot) vehicles[i].eesystem.getComponent("RLAutopilot").get();
            }
        }
        this.navigations = navigations;
        vehicleLengths = new double[vehicles.length];
        maxPathIndex = new int[vehicles.length];
        for (int i = 0; i < vehicles.length; i++) {
            vehicleLengths[i] = vehicles[i].properties.body.length;
            maxPathIndex[i] = -1;
        }
        currentSpeedLimits = new double[vehicles.length];

    }


    //calculate combined reward for all vehicles
    public float getReward() {
        float reward = 0;
        for (int i = 0; i < truePositions.length; i++) {
            reward += getRewardForVehicle(i);
        }

        return reward;
    }

    //Reward function to hold a velocity of 50km/h
    /*public float getRewardForVehicle(int carNumber){

        float reward = 0;
        double curSteering = steering[carNumber].getCurrentSteering();
        double deltaVelocity = Math.abs(50.0 - ((Double) trueVelocity[carNumber].get()).doubleValue());
        System.out.println("deltaVelocity: " + deltaVelocity);
        if(deltaVelocity >= 300) deltaVelocity = 40.0;

        if(deltaVelocity >= 40) {
            reward += -200;
        }else if(deltaVelocity >= 30 && deltaVelocity < 40) {
            reward += -150;
        }else if(deltaVelocity >= 20 && deltaVelocity < 30) {
            reward += -100;
        }else if(deltaVelocity >= 10 && deltaVelocity < 20) {
            reward += -50;
        }else if(deltaVelocity < 10 && deltaVelocity > 5) {
            reward += 150;
        }else if(deltaVelocity <= 5 && deltaVelocity > 1) {
            reward += 200;
        }else if(deltaVelocity <= 1) {
            reward += 300;
        }else {return (float) deltaVelocity;}

        System.out.println("Reward: " + reward);
        return reward;
    }
    */

    //Reward Function applicable to all cases
    public float getRewardForVehicle(int carNumber) {

        float reward = 0;
        List<StaticObject> objectCollision = vehicle[carNumber].getStaticCollisions();
        List<Vehicle> vehicleCollision = vehicle[carNumber].getVehicleCollisions();
        double curVelocity = ((Double) trueVelocity[carNumber].get()).doubleValue();
        double curSteering = steering[carNumber].getCurrentSteering();
        double deltaVelocity = Math.abs(15 - ((Double) trueVelocity[carNumber].get()).doubleValue()); //speed limit of 15 to prevent the agent of crashing

        if (deltaVelocity >= 300) deltaVelocity = 15; //if bug occurs that agent moves unrealistic fast
        if (curVelocity >= 40) { //if agent exceeds speed limit
            reward += -400;
        } else if (curVelocity < 1) {
            reward += -400;
        } else if (curVelocity < 10 && curVelocity > 5) {
            reward += 350;
        } else if (deltaVelocity <= 5 && deltaVelocity > 2) {
            reward += 250;
        } else {
            reward += 100;
        }

        if (curSteering <= -21.0 || curSteering >= 21.0) reward += -300; //steering of 21 is 0.7 * 30

        if (curVelocity >= 35.0 && (curSteering <= -21 || curSteering >= 21))
            reward += -800; //punishment: if too fast and too much steering

        Vec2[] traj = navigations[carNumber].getCurrentTraj();

        //calculate distance to nearest trajectory point
        Vec2 pos = (Vec2) truePositions[carNumber].get();

        double temp = 0;
        double distanceToSeg = 0;
        if (pos == null) return 0.f;
        double distance = Double.MAX_VALUE;
        for (int i = 0; i < traj.length - 1; i++) {
            SegmentPos currentSegment = new SegmentPos();
            currentSegment.initFromTraj(pos, traj[i], traj[i + 1]);
            if (currentSegment.projPos < 0) { //vehicle is in front on segment
                distanceToSeg = currentSegment.relPos.magnitude();
            } else if (currentSegment.projDistToEnd < 0) { //vehicle is behind segment
                distanceToSeg = currentSegment.endDis.magnitude();
            } else {
                distanceToSeg = currentSegment.dist;
            }
            if (distanceToSeg < distance) {
                temp = currentSegment.projPos;
                distance = distanceToSeg;
            }

        }
        //Punish if the distance to the trajectory is too great, otherwise reward
        if (distance > 5.0) {
            reward += -500;
        } else if (distance <= 1.5) {
            reward += 400;
        } else {
            reward += 100;
        }

        //punish collisions
        for (int i = 0; i < vehicleCollision.size() - 1; i++) { //punish collision with other vehicle
            reward += -600;
        }

        for (int i = 0; i < objectCollision.size() - 1; i++) { //punish collision with objects
            reward += -400;
        }

        //punish negative progress on the calculated path
        /*double progressReward = 0;
        int curPathIndex = navigations[carNumber].getCurrentPathIndex(pos);
        if(curPathIndex == -1) return 0.f;
        if(curPathIndex > maxPathIndex[carNumber]){ //den aktuell weitesten Fortschritt als maxPathIndex setzen
            maxPathIndex[carNumber] = curPathIndex;
            reward += 200;
        }
        if(curPathIndex < maxPathIndex[carNumber]){ //wenn plötzlich der aktuelle Fortschritt kleiner dann sozusagen negativer Fortschritt und punishment
            reward += -200;
        }*/

        return reward;
    }


    // SegmentPos taken from JavaAutopilot class for distance calculation
    private class SegmentPos {
        Vec2 posStart = new Vec2();
        Vec2 posEnd = new Vec2();
        Vec2 dir = new Vec2();
        Vec2 normal = new Vec2();
        Vec2 relPos = new Vec2();
        Vec2 endDis = new Vec2();
        Vec2 currentPosition;

        double length;
        double projPos;
        double orthoPos;
        double dist;
        double projDistToEnd;

        void initFromTraj(Vec2 currentPosition, Vec2 posStart, Vec2 posEnd) {
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
            } else {
                dir.set(Double.NaN, Double.NaN);
            }
            normal.set(-dir.y, dir.x);

            IPM.subtractTo(relPos, currentPosition, posStart);
            IPM.subtractTo(endDis, currentPosition, posEnd);
            projPos = IPM.dot(dir, relPos);
            orthoPos = IPM.dot(normal, relPos);
            dist = Math.abs(orthoPos);
            projDistToEnd = length - projPos;
        }
    }

    //get new speed limits
    private void updateCurrentSpeedLimit(int index) {
        if (speedLimitServices[index] == null) {
            currentSpeedLimits[index] = DEFAULT_SPEED_LIMIT;
        } else {
            currentSpeedLimits[index] = speedLimitServices[index].getSpeedLimit(0);
            if (currentSpeedLimits[index] <= 0) {
                currentSpeedLimits[index] = DEFAULT_SPEED_LIMIT;
            }
        }
    }
}
