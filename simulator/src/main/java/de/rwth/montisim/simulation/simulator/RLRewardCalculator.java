package de.rwth.montisim.simulation.simulator;

import de.rwth.montisim.simulation.commons.physicalvalue.*;
import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;
import de.rwth.montisim.simulation.eecomponents.speed_limit.SpeedLimitService;
import de.rwth.montisim.simulation.vehicle.physicalvalues.*;
import de.rwth.montisim.simulation.vehicle.Vehicle;


// This class is responsible for the reward calculation in 
// reinforcement learning scenarios

public class RLRewardCalculator{

    private final double DEFAULT_SPEED_LIMIT = 50.0; //in km/h
    private PhysicalValue[] truePositions;
    private PhysicalValue[] trueVelocity;
    private Navigation[] navigations;
    private SpeedLimitService[] speedLimitServices;
    private double[] vehicleLengths;
    private int[] maxPathIndex;
    private double[] currentSpeedLimits; //in km/h

    public RLRewardCalculator(Navigation[] navigations, Vehicle[] vehicles){
        truePositions = new TruePosition[vehicles.length];
        trueVelocity = new TrueVelocity[vehicles.length];
        speedLimitServices = new SpeedLimitService[vehicles.length];
        for(int i = 0; i<truePositions.length; i++){
            truePositions[i] = vehicles[i].physicalValues.getPhysicalValue("true_position");
            trueVelocity[i] = vehicles[i].physicalValues.getPhysicalValue("true_velocity"); 
            if(!vehicles[i].eesystem.getComponent("SpeedLimit").isPresent()){
                speedLimitServices[i] = null;
            }
            else{
                speedLimitServices[i] = (SpeedLimitService) vehicles[i].eesystem.getComponent("SpeedLimit").get();
            }
        }
        this.navigations = navigations;
        vehicleLengths = new double[vehicles.length];
        maxPathIndex = new int[vehicles.length];
        for(int i = 0; i<vehicles.length; i++){
            vehicleLengths[i] = vehicles[i].properties.body.length;
            maxPathIndex[i] = -1;
        }
        currentSpeedLimits = new double[vehicles.length];

    }

    //calculate combined reward for all vehicles
    public float getReward(){
        float reward  = 0;
        for(int i = 0; i<truePositions.length; i++){
            reward += getRewardForVehicle(i);
        }       

        return reward;
    }

    //provide reward for individual vehicle
    public float getRewardForVehicle(int carNumber){
        if(carNumber < 0 || carNumber >= truePositions.length) return 0.f;
        updateCurrentSpeedLimit(carNumber);
        if(truePositions[carNumber] == null || navigations[carNumber] == null || trueVelocity[carNumber]==null) return 0.f;
        Vec2 pos = (Vec2) truePositions[carNumber].get();
        if(pos == null) return 0.f;
        Vec2[] traj = navigations[carNumber].getCurrentTraj();

        //calculate distance to nearest trajectory point
        double distance = Double.MAX_VALUE;
        for(int i = 0; i<traj.length-1; i++){
            SegmentPos currentSegment = new SegmentPos();
            currentSegment.initFromTraj(pos, traj[i],traj[i+1]);
            if(currentSegment.dist < distance) distance = currentSegment.dist;
        }
        //calculate difference to maximal allowed velocity 
        double deltaVelocity = Math.abs(currentSpeedLimits[carNumber] - ((Double) trueVelocity[carNumber].get()).doubleValue());
        double crashReward = 0;

        //punish collisions
        for(int i = 1; i<truePositions.length;i++){
            double vehicleDistance = pos.distance((Vec2) truePositions[(carNumber + i)%truePositions.length].get());
            if(vehicleDistance <= vehicleLengths[carNumber] || vehicleDistance <= vehicleLengths[(carNumber + i)%truePositions.length]){
                crashReward += (double) -100;
            }
        }

        //punish negative progress on the calculated path
        double progressReward = 0;
        int currentPathIndex = navigations[carNumber].getCurrentPathIndex(pos);
        if(currentPathIndex == -1) return 0.f;
        if(currentPathIndex > maxPathIndex[carNumber]){
            maxPathIndex[carNumber] = currentPathIndex;
        }
        if(currentPathIndex < maxPathIndex[carNumber]){
            progressReward += (double) -10;
        }

        return (float) (- (distance * distance) - deltaVelocity + progressReward + crashReward);

    }

    // SegmentPos taken from JavaAutopilot class for distance calculation
    private class SegmentPos {
        Vec2 posStart = new Vec2();
        Vec2 posEnd = new Vec2();
        Vec2 dir = new Vec2();
        Vec2 normal = new Vec2();
        Vec2 relPos = new Vec2();
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
            projPos = IPM.dot(dir, relPos);
            orthoPos = IPM.dot(normal, relPos);
            dist = Math.abs(orthoPos);
            projDistToEnd = length - projPos;
        }
    }

    //get new speed limits
    private void updateCurrentSpeedLimit(int index){
        if(speedLimitServices[index] == null){
            currentSpeedLimits[index] = DEFAULT_SPEED_LIMIT;
        }
        else{
            currentSpeedLimits[index] = speedLimitServices[index].getSpeedLimit(0);
            if(currentSpeedLimits[index] <= 0){
                currentSpeedLimits[index] = DEFAULT_SPEED_LIMIT;
            }
        }
    }
}
