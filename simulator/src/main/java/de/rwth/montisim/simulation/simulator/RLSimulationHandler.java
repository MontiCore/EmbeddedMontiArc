package de.rwth.montisim.simulation.simulator;

import de.rwth.montisim.commons.map.Path;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.simulation.commons.util.CollisionLogWriter;
import de.rwth.montisim.simulation.eecomponents.autopilots.*;
import de.rwth.montisim.simulation.eecomponents.lidar.Lidar;
import de.rwth.montisim.simulation.eecomponents.speed_limit.SpeedLimitService;
import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.visualization.rl.RLVisualizer;
import de.rwth.montisim.simulation.vehicle.navigation.*;
import de.rwth.montisim.simulation.vehicle.physicalvalues.*;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoalProperties;
import de.rwth.montisim.simulation.vehicle.task.TaskProperties;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;


import java.awt.event.*;
import java.io.File;
import java.lang.Double;
import java.lang.Thread;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collection;
import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.Arrays;

// This class is responsible for the training and playing with 
// reinforcement learning agents

public class RLSimulationHandler extends AbstractNodeMain{

    //RL settings
    private boolean distributed = false;
    private boolean randomize = false;
    private boolean PLAYMODE = false;
    private boolean miniStep = false;
    private String selfPlay_mode = ".";

    private final long ACCESS_TIME = 100; //simulated time between 2 control signals in ms
    private int update_iterations = 1; //number of update calls required for one step
    private final int DEFAULT_STATE_LENGTH = 25;
    private final long seed = 48965l;
    private int lidar_offset = 0;
    private int speed_limit_offset = 0;

    public Simulator simulator = null;
    private SimulationConfig config;
    Instant simulationTime;
    private OsmMap map;
    private World world;
    private Pathfinding pathfinding;
    private Vehicle[] vehiclesArray;
    private RLAutopilot[] autopilots;
    private Navigation[] navigations;
    private TruePosition[] truePositions;
    private TrueVelocity[] trueVelocity;
    private TrueCompass[] trueCompass; 
    private RLRewardCalculator rewardCalc;
    private Random rndGen = new Random();

    private int activeVehicle = 0; //Round Robin variable
    private int trainedVehicle = 0; //denotes which vehicle is currently trained, all other vehicles are simulated by the self-play agent
    
    private List<float[]> decentralizedActionsList = new ArrayList<float[]>(); //list to save actions send by the self-play agent
    private boolean receiveActions = true; 
    private boolean setupComplete = false; //simulation setup
    private boolean finished = true; //prevent race condition
    private int currentVehicle = 0;

    private RLVisualizer viz;

    private Lidar[] lidars = null;
    private SpeedLimitService[] speedLimitServices = null;

    private boolean in_termination = true;
    private boolean in_reset = false;
    private boolean done = true;
    private int episodeCounter = 0;
    private NodeMainExecutor nodeExecutor;

    private float[][] vehicleStates;
    private int stateLength;

    private class Result{
        float reward = 0f;
        float[] state = null;
        boolean terminated = true;

        public Result (float reward, float[] state, boolean terminated){
            this.reward = reward;
            this.state = state;
            this.terminated = terminated;
        }

        public Result(){}
    }

    public RLSimulationHandler (SimulationConfig config, Instant simulationTime, OsmMap map, RLVisualizer viz, NodeMainExecutor nodeExecutor){
        this.nodeExecutor = nodeExecutor;
        this.config = config;
        this.simulationTime = simulationTime;
        this.map = map;
        this.viz = viz;
        rndGen.setSeed(seed);
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("simulator_rl/simulationHandler");
    }

    //initialize publishers and subscribers
    @Override
    public void onStart(ConnectedNode connectedNode){
    
    final String state_topic = "sim/state";
    final String terminate_topic = "/sim/terminal";
    final String step_topic = "/sim/step";
    final String reset_topic = "/sim/reset";
    final String reward_topic = "/sim/reward";
    
	//initialize subscribers and publishers
	Subscriber<std_msgs.Float32MultiArray> action_subscriber = connectedNode.newSubscriber(step_topic, std_msgs.Float32MultiArray._TYPE);
    Subscriber<std_msgs.Bool> reset_subscriber = connectedNode.newSubscriber(reset_topic, std_msgs.Bool._TYPE);

    final Publisher<std_msgs.Float32MultiArray> state_publisher = connectedNode.newPublisher(state_topic, std_msgs.Float32MultiArray._TYPE);
    final Publisher<std_msgs.Bool> terminate_publisher = connectedNode.newPublisher(terminate_topic, std_msgs.Bool._TYPE);
    final Publisher<std_msgs.Float32> reward_publisher = connectedNode.newPublisher(reward_topic, std_msgs.Float32._TYPE);

    //initialize publishers and subscribers for the self-play agent
    //reward topic not required since the self-play agent does not process it
    final String state2_topic = "sim/state2";
    final String step2_topic = "/sim/step2";
    final String reset2_topic = "/sim/reset2";
    final String terminate2_topic = "/sim/terminal2";

    Subscriber<std_msgs.Float32MultiArray> action2_subscriber = connectedNode.newSubscriber(step2_topic, std_msgs.Float32MultiArray._TYPE);
    Subscriber<std_msgs.Bool> reset2_subscriber = connectedNode.newSubscriber(reset2_topic, std_msgs.Bool._TYPE);

    final Publisher<std_msgs.Float32MultiArray> state2_publisher = connectedNode.newPublisher(state2_topic, std_msgs.Float32MultiArray._TYPE); //braucht man den wirklich oder ist der nicht überflüssig?
    final Publisher<std_msgs.Bool> terminate2_publisher = connectedNode.newPublisher(terminate2_topic, std_msgs.Bool._TYPE);


    //called when self-play agent sends an action
    action2_subscriber.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>() {
      @Override
      public void onNewMessage(std_msgs.Float32MultiArray action) {
            if(in_termination || in_reset) return;

            if(setupComplete) {
                if(PLAYMODE == false) { //training mode activated
                    if(finished) {
                        finished = false;
                        if(currentVehicle < vehiclesArray.length-1){
                            currentVehicle += 1;
                        }
                        if(currentVehicle == trainedVehicle && currentVehicle < vehiclesArray.length-1) { //make sure trained and current vehicle do not overlap
                            currentVehicle += 1;
                        }

                        if(receiveActions) { 
                            decentralizedActionsList.add(action.getData()); //save received actions
                        }
                        if(decentralizedActionsList.size() >= vehiclesArray.length-1) { //only n-1 actions required by the self-play agent for n vehicles
                            receiveActions = false;
                        }

                        //publish next state to the self-play agent 
                        //make sure currentVehicle is not the trainedVehicle and the the number of vehicles is over 2, otherwise dont publish again
                        if(currentVehicle <= vehiclesArray.length-1 && currentVehicle != trainedVehicle && !(vehiclesArray.length-1 == 1)){
                            std_msgs.Float32MultiArray state2 = state2_publisher.newMessage();
                            state2.setData(getDistributedState(currentVehicle));
                            std_msgs.Bool terminated2 = terminate2_publisher.newMessage();

                            terminated2.setData(false);
                            finished = true;

                            state2_publisher.publish(state2);
                            terminate2_publisher.publish(terminated2);
                        }
                        else{
                            finished = true;
                        }
                    }

                }
                else{ //execution mode activated
                    //publish state for every vehicle to the self-play agent
                    for(int i = currentVehicle; i <= vehiclesArray.length-1; i++) {
                        std_msgs.Float32MultiArray state2 = state2_publisher.newMessage();
                        state2.setData(getDistributedState(i));
                        std_msgs.Bool terminated2 = terminate2_publisher.newMessage();
                        //boolean term = getSimTermination();
                        terminated2.setData(false);
                        state2_publisher.publish(state2);
                        terminate2_publisher.publish(terminated2);
                    }

                    if(receiveActions) { 
                        decentralizedActionsList.add(action.getData());
                    }
                    
                    if(decentralizedActionsList.size() >= vehiclesArray.length) {
                        receiveActions= false;
                    }

                    //if action for every vehicle received simulate next step
                    if(decentralizedActionsList.size() == vehiclesArray.length) { 

                        Result result = step(action.getData()); 
                    
                        std_msgs.Float32MultiArray state = state2_publisher.newMessage();
                        state.setData(result.state);
                        std_msgs.Bool terminated = terminate2_publisher.newMessage();
                        terminated.setData(result.terminated);

                        if(result.terminated) in_termination = true;
                        
                        state2_publisher.publish(state);
                        terminate2_publisher.publish(terminated);
                    }
            
                }


            }
            else{
                //System.out.println("Waiting");
            }
          
        }
    });
    
	//called when agent gives action for next step
    action_subscriber.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>() {
      @Override
      public void onNewMessage(std_msgs.Float32MultiArray action) {
		if(in_termination || in_reset) return;
        if(distributed && PLAYMODE == false && !miniStep) {
            if(trainedVehicle == currentVehicle) { //make sure to dont mix up trained and current vehicle
                if(currentVehicle < vehiclesArray.length-1) {
                    currentVehicle += 1;
                    std_msgs.Float32MultiArray state2 = state2_publisher.newMessage();
                    state2.setData(getDistributedState(currentVehicle));
                    std_msgs.Bool terminated2 = terminate2_publisher.newMessage();
                    terminated2.setData(false);
                    /*while(decentralizedActionsList.size() > 0) { //extra verfication that list is empty for new actions
                        decentralizedActionsList.remove(0);
                    }*/
                    finished = true;
                    receiveActions = true;
                    state2_publisher.publish(state2);
                    terminate2_publisher.publish(terminated2);
                }

            }
            else {
                if(currentVehicle <= vehiclesArray.length-1) { 
                    std_msgs.Float32MultiArray state2 = state2_publisher.newMessage();
                    state2.setData(getDistributedState(currentVehicle));
                    std_msgs.Bool terminated2 = terminate2_publisher.newMessage();
                    terminated2.setData(false);
                    /*while(decentralizedActionsList.size() > 0) { //extra verfication that list is empty for new actions
                        decentralizedActionsList.remove(0);
                    }*/
                    finished = true;
                    receiveActions = true;
                    state2_publisher.publish(state2);
                    terminate2_publisher.publish(terminated2);
                }
            }

            while(true) { //wait until all actions of self-play agent are send
                try{
                    Thread.sleep(0);
                } catch (InterruptedException ex) {}

                if(decentralizedActionsList.size() >= vehiclesArray.length-1) break;              
            }

        }

        Result result = step(action.getData()); //simulate next step
            
        std_msgs.Float32MultiArray state = state_publisher.newMessage(); //return reward, state and etc.
        state.setData(result.state);
        std_msgs.Bool terminated = terminate_publisher.newMessage();
        terminated.setData(result.terminated);
        std_msgs.Float32 reward = reward_publisher.newMessage();
        reward.setData(result.reward);
    
        if(result.terminated) in_termination = true;
                    
        state_publisher.publish(state);
        terminate_publisher.publish(terminated);
        reward_publisher.publish(reward);

    } 

    });

	//called when agent calls for a reset
    reset_subscriber.addMessageListener(new MessageListener<std_msgs.Bool>() { //TODO: hier auch noch anderes Vorgehen für decentralized approach hinzufügen
      @Override
      public void onNewMessage(std_msgs.Bool reset) {
		if(reset.getData() && in_termination){
		  in_reset = true;
		  Result result = reset();

		  std_msgs.Float32MultiArray state = state_publisher.newMessage();
		  state.setData(result.state);
		  std_msgs.Bool terminated = terminate_publisher.newMessage();
		  terminated.setData(result.terminated);
		  std_msgs.Float32 reward = reward_publisher.newMessage();
		  reward.setData(result.reward);

		  state_publisher.publish(state);
		  terminate_publisher.publish(terminated);
		  reward_publisher.publish(reward);
		  in_termination = false;
		  in_reset = false;
		}
        
      }

    });

    //called when self-play agent calls for a reset
    reset2_subscriber.addMessageListener(new MessageListener<std_msgs.Bool>() { //TODO: hier auch noch anderes Vorgehen für decentralized approach hinzufügen
        @Override
        public void onNewMessage(std_msgs.Bool reset) {
          if(reset.getData() && in_termination){
            in_reset = true;
            Result result = reset();
  
            std_msgs.Float32MultiArray state = state2_publisher.newMessage();
            state.setData(result.state);
            std_msgs.Bool terminated = terminate2_publisher.newMessage();
            terminated.setData(result.terminated);
            std_msgs.Float32 reward = reward_publisher.newMessage();
            reward.setData(result.reward);
  
            state2_publisher.publish(state);
            terminate2_publisher.publish(terminated);
            reward_publisher.publish(reward);
            in_termination = false;
            in_reset = false;
          }
          
        }
  
      });

        //if the agent is not trained the environment has to provide the
        //initial messages
        if(PLAYMODE==true){
          try{
              Thread.sleep(5000);
          } catch (InterruptedException ex) {}
		  in_reset = true;
          currentVehicle = 0;
		  Result result = reset();

		  std_msgs.Float32MultiArray state = state_publisher.newMessage();
		  state.setData(result.state);
		  std_msgs.Bool terminated = terminate_publisher.newMessage();
		  terminated.setData(result.terminated);
		  std_msgs.Float32 reward = reward_publisher.newMessage();
		  reward.setData(result.reward);

		  state_publisher.publish(state);
		  terminate_publisher.publish(terminated);
		  reward_publisher.publish(reward);
		  in_termination = false;
		  in_reset = false;
        }
        else {
            currentVehicle = 0;
        }
    }

    @Override
    public void onShutdown(Node node){
        if(simulator != null){
            simulator.destroy();
            simulator = null;
        }
    }

    public Result reset(){
        ++this.episodeCounter;
        return this.setup();
    }

    //starts a new simulation and initializes all variables
    private Result setup(){
        if(simulator != null){
            simulator.destroy();
            simulator = null;
        }
        if(randomize){
            randomizeScenario();
        }
        if(viz!= null){ 
            viz.clearRenderer();
        }
        world = new OsmToWorldLoader(map).getWorld();
        pathfinding = new PathfindingImpl(world);
        simulator = config.build(world, pathfinding, map);
        vehiclesArray = simulator.getVehicles().toArray(Vehicle[]::new);
        if(viz != null){
            viz.simTime = simulationTime;
            viz.setup(world, pathfinding);
        }

        if(config.tick_duration.toMillis() <= 100l){
            update_iterations = (int) (100l/config.tick_duration.toMillis());
        }

        //parse all required components
        autopilots = new RLAutopilot[vehiclesArray.length];
        for(int i = 0; i<autopilots.length; i++){
            autopilots[i] = (RLAutopilot) vehiclesArray[i].eesystem.getComponent("RLAutopilot").get();
        }

        navigations = new Navigation[autopilots.length];
        for(int i = 0;i<navigations.length;i++){
            navigations[i] = (Navigation) vehiclesArray[i].eesystem.getComponent("Navigation").get();
        }

        truePositions = new TruePosition[autopilots.length];
        trueVelocity = new TrueVelocity[autopilots.length];
        trueCompass = new TrueCompass[autopilots.length];
        lidars = new Lidar[autopilots.length];
        speedLimitServices = new SpeedLimitService[autopilots.length];
        for(int i = 0; i<truePositions.length;i++){
            truePositions[i] = (TruePosition) vehiclesArray[i].physicalValues.getPhysicalValue("true_position");
            trueVelocity[i] = (TrueVelocity) vehiclesArray[i].physicalValues.getPhysicalValue("true_velocity");
            trueCompass[i] = (TrueCompass) vehiclesArray[i].physicalValues.getPhysicalValue("true_compass");
            if(vehiclesArray[i].eesystem.getComponent("Lidar").isPresent()){
                lidars[i] = (Lidar) vehiclesArray[i].eesystem.getComponent("Lidar").get();
                lidar_offset = lidars[i].getMessageLength();
            }
            if(vehiclesArray[i].eesystem.getComponent("SpeedLimit").isPresent()){
                speedLimitServices[i] = (SpeedLimitService) vehiclesArray[i].eesystem.getComponent("SpeedLimit").get();
                speed_limit_offset = 1;
            }
        }

        stateLength = DEFAULT_STATE_LENGTH + lidar_offset + speed_limit_offset;
        vehicleStates = new float[autopilots.length][stateLength];

        //update simulation until all values are assigned
        while(anyStateNull()){
            TimeUpdate tu = new TimeUpdate(simulationTime, config.tick_duration);
            simulator.update(tu);
            simulationTime = tu.newTime;
        }
        activeVehicle = 0;
        receiveActions = true;

        if(selfPlay_mode == "afterEpisode") { // afterEpisode option can be selected via CLI
            trainedVehicle = (trainedVehicle +1)%vehiclesArray.length; //pick the next vehicle as the trained vehicle
        } 
        updateStatePackets();
        float[] simState = getState();
        rewardCalc = new RLRewardCalculator(navigations, vehiclesArray);
        float init_reward = rewardCalc.getReward();
        boolean simTermination = this.getSimTermination();

        //check if all vehicles found a path, if not, restart simulation
        for(int i = 0; i<navigations.length; i++){
            if(navigations[i].getCurrentPath().get().getLength() == 0)
                return setup();
        }
        setupComplete = true;

        return new Result(init_reward, simState, simTermination);
    }


    private Result step(float[] action){

        //self-play and training mode
        if(distributed && PLAYMODE == false && !miniStep) { 
            int listCounter = 0; //makes sure that right action out of the decentralizedActionsList is selected
            for(int i = 0; i<=autopilots.length-1; i++) {

                if(i == trainedVehicle) {
                    setAction(action); //set action for the trained vehicle
                } else if( i != trainedVehicle && decentralizedActionsList!=null && decentralizedActionsList.size()!=0) {
                    if(decentralizedActionsList.size() >= i) { //set self-play actions
                        setDistributedAction(decentralizedActionsList.get(listCounter), i);
                        listCounter +=1;
                    }
                    else if(decentralizedActionsList.size() < i && decentralizedActionsList.size() >= 1){
                        setDistributedAction(decentralizedActionsList.get(0), i);
                    }    
                }
            }
            listCounter = 0;

        }
        //set action for all vehicles in self-play execution mode
        else if(distributed && PLAYMODE == true && !miniStep) {
            if(decentralizedActionsList!=null && decentralizedActionsList.size()!=0) {
                for(int i=0; i<= decentralizedActionsList.size()-1; i++) {
                    setDistributedAction(decentralizedActionsList.get(i),i);
                
                }
            }
        }
        //set action for single-vehicle and centralized case
        else{
            setAction(action);
        }
        

        if(simulator == null) return new Result();
        
        if(!miniStep) {
            for(int i = 0; i<update_iterations; i++){
                if(simulator != null){
                    TimeUpdate tu = new TimeUpdate(simulationTime, config.tick_duration);
                    simulator.update(tu);
                    simulationTime = tu.newTime;
                }
            }
        }
        else{
            for(int i = 0; i<update_iterations; i++){
                if(simulator != null){
                    TimeUpdate tu = new TimeUpdate(simulationTime, config.tick_duration.dividedBy((long) vehiclesArray.length));
                    simulator.update(tu);
                    simulationTime = tu.newTime;
                }
            }
        }
        

        updateStatePackets();
        float[] simState = getState();
        float step_reward;

        if(!distributed){
            step_reward = rewardCalc.getReward();
        }
        else if(miniStep) {
            step_reward = rewardCalc.getRewardForVehicle(activeVehicle);
        }
        else{
            step_reward = rewardCalc.getRewardForVehicle(trainedVehicle); //calculate reward only for trained vehicle in self-play mode
        }

        boolean simTermination = this.getSimTermination();
        activeVehicle = (activeVehicle + 1)%vehiclesArray.length;

        if(distributed & !miniStep) {
   
            while(decentralizedActionsList.size() > 0) { //clear self-play action list for next step
                decentralizedActionsList.remove(0);
            }
        }

        currentVehicle=0;

        //afterStep option is selected in CLI
        if(selfPlay_mode == "afterStep") {
             trainedVehicle = (trainedVehicle +1)%vehiclesArray.length; //pick next vehicle as the trainedVehicle
        }

        receiveActions = true;

        if(viz != null){
            try{viz.simTime = simulationTime;
            viz.redraw();
            }
            catch(ConcurrentModificationException ignore) {}
        }
        if(simTermination && config.collision_mode.equals("LOG_COLLISIONS")){
            CollisionLogWriter.addCollisions(simulator.collisionHistory, config.start_time, true, !PLAYMODE, episodeCounter);
        }
        done = true;

        return new Result(step_reward, simState, simTermination);
    } 

    private void updateStatePackets(){
        for(int i = 0; i<autopilots.length; i++){
            autopilots[i].updateStatePacket();
        }
    }

    private void wait_done(){
        while(!done);
        done = false;
        return;
    }

    private boolean getSimTermination(){
        if(simulator.status() == TaskStatus.RUNNING) return false;
        else return true;
    }

    //return the state array that is published on the state topic
    private float[] getState(){
        int vehicleCount = autopilots.length;
        int statePacketLength = autopilots[0].getStatePacket().length;
        updateVehicleStates();

        float[] result;
        if(!distributed){
            result = new float[vehicleCount * stateLength];

            for(int i = 0; i<vehicleCount; i++){
                for(int j = 0; j<stateLength; j++){
                    result[i*stateLength + j] = vehicleStates[i][j];
                }
            }
        }
        else if(miniStep){
            result = new float[stateLength + (vehicleCount-1) * statePacketLength];
            for(int i = 0; i<stateLength; i++){
                result[i] = vehicleStates[activeVehicle][i];
            }
            for(int i = 1; i< vehicleCount; i++){
                for(int j = 0; j < statePacketLength; j++){
                    result[stateLength + (i-1)*statePacketLength + j] = autopilots[(activeVehicle + i) % vehicleCount].getStatePacket()[j];
                }
            }
        }
        else{
            result = new float[stateLength + (vehicleCount-1) * statePacketLength];
            for(int i = 0; i<stateLength; i++){
                result[i] = vehicleStates[trainedVehicle][i]; //return state to EMADL agent
            }
            for(int i = 1; i< vehicleCount; i++){
                for(int j = 0; j < statePacketLength; j++){
                    result[stateLength + (i-1)*statePacketLength + j] = autopilots[(trainedVehicle + i) % vehicleCount].getStatePacket()[j];
                }
            }
        }

        return result;
    }

    //method to return the state to the self-play vehicle
    private float[] getDistributedState(int index) {
        int vehicleCount = autopilots.length;
        int stateLength = autopilots[0].state.length;
        int statePacketLength = autopilots[0].getStatePacket().length;
        updateVehicleStates();
        float[] result;
        result = new float[stateLength + (vehicleCount-1) * statePacketLength];
        for(int i = 0; i<stateLength; i++){
            result[i] = vehicleStates[index][i];
        }
        for(int i = 1; i< vehicleCount; i++){
            for(int j = 0; j < statePacketLength; j++){
                result[stateLength + (i-1)*statePacketLength + j] = autopilots[(index + i) % vehicleCount].getStatePacket()[j];
            }
        }
        return result;

    }

    //set the combined action from all vehicles
    private void setAction(float[] action){

        int vehicleCount = autopilots.length;
        int actionLength =  action.length / vehicleCount; //assume that every vehicle has same action space

        if(!distributed){ //single vehicle or centralized
            for(int i = 0; i<vehicleCount; i++){
                float[] result = new float[actionLength];
                for(int j = 0; j<actionLength; j++){
                    result[j] = action[i * actionLength + j];
                }
                autopilots[i].action = result;
            } 
        }
        else if(miniStep){
            
            autopilots[activeVehicle].action = action; //mini-step approach
        }
        else{
            
            autopilots[trainedVehicle].action = action; //self-play approach
        }

        return;
    }

    //set action of the self play agent
    private void setDistributedAction(float[] action, int index) {
        autopilots[index].action = action;
    }

    private boolean anyStateNull(){
        for(int i = 0; i<autopilots.length; i++){
            if(autopilots[i].state == null) return true;
        }
        return false;
    }

    //assign random start and target coordinates for all vehicles
    //addtional orientation allignment with first segment

    private void randomizeScenario(){
        if(pathfinding == null) return;
        VehicleProperties[] properties;
        properties = config.cars.toArray(new VehicleProperties[0]);
        Vec2[] startCoords = new Vec2[properties.length];
        Vec2[] targetCoords = new Vec2[properties.length];
        double[] startOrientations = new double[properties.length];
        int x_boundary = (int) world.maxCorner.at(0);
        int y_boundary = (int) world.maxCorner.at(1);

        for(int i = 0; i < properties.length; i++){
            Path path = new Path(0);
            do{
            startCoords[i] = new Vec2(rndGen.nextInt()%x_boundary,rndGen.nextInt()%y_boundary);
            targetCoords[i] = new Vec2(rndGen.nextInt()%x_boundary,rndGen.nextInt()%y_boundary);
            try{
                path = pathfinding.findShortestPath(startCoords[i],targetCoords[i]);
            } catch (Exception e) {}
            } while(path.getLength()<=0);

            path.get(1,startCoords[i]);
            path.get(path.getLength()-2,targetCoords[i]);

            Vec2 dir = new Vec2();
            Vec2 secondPoint = new Vec2();
            path.get(2,secondPoint);
            IPM.subtractTo(dir,secondPoint , startCoords[i]);
            double length = dir.magnitude();
            if (length > 0.001) {
                IPM.multiply(dir, 1 / length);
            } else {
                dir.set(Double.NaN, Double.NaN);
            }
            startOrientations[i] = (180* Math.acos(dir.x) * Math.signum(dir.y))/Math.PI;
        }

        for(int i = 0; i<properties.length;i++){
            TaskProperties task = new TaskProperties();
            task.addGoal(new PathGoalProperties()
                .reach(targetCoords[i])
                .withinRange(10)
                .eventually());
            properties[i].task = task;
            properties[i].start_pos = Optional.of(startCoords[i]);
            properties[i].start_orientation = startOrientations[i];
        }
    }

    public Simulator getSim(){
        return simulator;
    }

    //update the array containing the states of all vehicles
    public void updateVehicleStates(){
        for(int i = 0; i<autopilots.length;i++){
            for(int j = 0; j<stateLength;j++){
                vehicleStates[i][j] = 0.f;
            }
            int trajectoryLength = navigations[i].getCurrentTrajSize();
            for(int j = 0; j<trajectoryLength; j++){
                vehicleStates[i][j] = (float) navigations[i].getCurrentTraj()[j].at(0);
                vehicleStates[i][10+j] = (float) navigations[i].getCurrentTraj()[j].at(1);
            }
            vehicleStates[i][20] = (float) trajectoryLength;
            vehicleStates[i][21] = (float) ((Vec2) truePositions[i].get()).at(0);
            vehicleStates[i][22] = (float) ((Vec2) truePositions[i].get()).at(1);
            vehicleStates[i][23] = (float) ((Double) trueCompass[i].get()).doubleValue();
            vehicleStates[i][24] = (float) ((Double) trueVelocity[i].get()).doubleValue();
            if(speed_limit_offset >0){
                vehicleStates[i][25] = (float) speedLimitServices[i].getSpeedLimit(0);
            }
            if(lidar_offset >0){
                for(int j = 0; j<lidar_offset; j++){
                    vehicleStates[i][DEFAULT_STATE_LENGTH + speed_limit_offset + j] = (float) lidars[i].getLidarValue(j);
                }
            }

        }
    }

    // set reinforcement learning settings
    public void setSettings(boolean distributed, boolean randomize, boolean play, boolean miniStep, String selfPlay_mode){
        this.distributed = distributed;
        this.randomize = randomize;
        this.PLAYMODE = play;
        this.miniStep = miniStep;
        this.selfPlay_mode = selfPlay_mode;
    }

}