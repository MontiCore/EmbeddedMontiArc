package de.rwth.montisim.simulation.simulator;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.simulation.commons.util.CollisionLogWriter;
import de.rwth.montisim.simulation.commons.util.VelocityLogWriter;
import de.rwth.montisim.simulation.eecomponents.autopilots.RLAutopilot;
import de.rwth.montisim.simulation.eecomponents.lidar.Lidar;
import de.rwth.montisim.simulation.eecomponents.speed_limit.SpeedLimitService;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.osmmap.OsmToWorldLoader;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.randomization.RandomRandomizationPropertiesPicker;
import de.rwth.montisim.simulation.simulator.randomization.RandomizationProperties;
import de.rwth.montisim.simulation.simulator.randomization.RandomizationStrategy;
import de.rwth.montisim.simulation.simulator.rewards.DefaultRewardFunctionProperties;
import de.rwth.montisim.simulation.simulator.rewards.RewardFunction;
import de.rwth.montisim.simulation.simulator.visualization.rl.RLVisualizer;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueCompass;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TruePosition;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;

import java.time.Instant;
import java.util.*;

// This class is responsible for the training and playing with
// reinforcement learning agents

public class RLSimulationHandler {

  private final long ACCESS_TIME = 100; //simulated time between 2 control signals in ms
  private final int DEFAULT_STATE_LENGTH = 25;
  public Simulator simulator = null;
  Instant simulationTime;
  //RL settings
  private boolean distributed = false;
  private boolean randomize = false;
  private boolean PLAYMODE = false;
  private boolean miniStep = false;
  private String selfPlay_mode = ".";
  private int update_iterations = 1; //number of update calls required for one step
  private int lidar_offset = 0;
  private int speed_limit_offset = 0;
  private final SimulationConfig config;
  private final OsmMap map;
  private World world;
  private Pathfinding pathfinding;
  private Vehicle[] vehiclesArray;
  private RLAutopilot[] autopilots;
  private Navigation[] navigations;
  private TruePosition[] truePositions;
  private TrueVelocity[] trueVelocity;
  private TrueCompass[] trueCompass;
  private RewardFunction rewardFunction;

  private int activeVehicle = 0; //Round Robin variable
  private int trainedVehicle = 0; //denotes which vehicle is currently trained, all other vehicles are simulated by the self-play agent

  private final List<float[]> decentralizedActionsList = new ArrayList<float[]>(); //list to save actions send by the self-play agent
  private boolean receiveActions = false;
  private boolean setupComplete = false; //simulation setup
  private boolean finished = true; //prevent race condition
  private int currentVehicle = 0;

  private final RLVisualizer viz;

  private Lidar[] lidars = null;
  private SpeedLimitService[] speedLimitServices = null;

  private boolean in_termination = true;
  private boolean in_reset = false;
  private boolean done = true;
  private int episodeCounter = 0;
  private int stepCounter = 0;

  private float[][] vehicleStates;
  private int stateLength;

  private long cppInterface;
  private float[] train_action;
  private boolean in_action = false;

  public RLSimulationHandler(SimulationConfig config, Instant simulationTime, OsmMap map, RLVisualizer viz) {
    this.config = config;
    this.simulationTime = simulationTime;
    this.map = map;
    this.viz = viz;
  }

  //called when self-play agent sends an action
  public void action2(float[] action) {
    if (in_termination || in_reset) {
      return;
    }
    if (setupComplete) {
      if (PLAYMODE == false) { //training mode activated
        if (finished && !receiveActions && in_action) {
          do_step();
          return;
        }
        if (finished) {
          finished = false;
          if (currentVehicle < vehiclesArray.length - 1) {
            currentVehicle += 1;
          }
          if (currentVehicle == trainedVehicle && currentVehicle < vehiclesArray.length - 1) { //make sure trained and current vehicle do not overlap
            currentVehicle += 1;
          }
          if (receiveActions) {
            decentralizedActionsList.add(action); //save received actions
          }
          if (decentralizedActionsList.size() >= vehiclesArray.length - 1) { //only n-1 actions required by the self-play agent for n vehicles
            receiveActions = false;
          }

          //publish next state to the self-play agent
          //make sure currentVehicle is not the trainedVehicle and the the number of vehicles is over 2, otherwise dont publish again
          if (currentVehicle <= vehiclesArray.length - 1 && currentVehicle != trainedVehicle && !(vehiclesArray.length - 1 == 1)) {
            finished = true;
            publishNonTrainMessage(cppInterface, getDistributedState(currentVehicle), false);
          }
          else {
            finished = true;
          }
          finished = true;
        }
      }
      else { //execution mode activated
        if (receiveActions) {
          decentralizedActionsList.add(action);
        }

        if (decentralizedActionsList.size() >= vehiclesArray.length) {
          receiveActions = false;
        }
        else {
          currentVehicle++;
          publishNonTrainMessage(cppInterface, getDistributedState(currentVehicle), false);
        }

        //if action for every vehicle received simulate next step
        if (decentralizedActionsList.size() == vehiclesArray.length) {
          Result result = step(action);
          if (result.terminated)
            in_termination = true;
          publishNonTrainMessage(cppInterface, result.state, result.terminated);
        }
      }
    }
  }

  //called when agent gives action for next step
  public void action(float[] action) {
    if (in_termination || in_reset || in_action) {
      return;
    }
    in_action = true;
    if (distributed && PLAYMODE == false && !miniStep) {
      if (currentVehicle < vehiclesArray.length - 1 || (trainedVehicle != currentVehicle && currentVehicle <= vehiclesArray.length - 1)) {
        if (trainedVehicle == currentVehicle)
          currentVehicle++;
        finished = true;
        receiveActions = true;

        publishNonTrainMessage(cppInterface, getDistributedState(currentVehicle), false);
      }
      train_action = action;
      if (decentralizedActionsList.size() >= vehiclesArray.length - 1)
        do_step();

    }
    else {
      train_action = action;
      do_step();
    }
  }

  private void do_step() {
    Result result = step(train_action);
    if (result.terminated)
      in_termination = true;
    in_action = false;
    publishTrainMessage(cppInterface, result.state, result.terminated, result.reward);
  }

  //called when agent calls for a reset
  public void reset1(boolean reset) {
    if (reset && in_termination) {
      in_reset = true;
      Result result = reset();

      in_termination = false;
      in_reset = false;
      if (!PLAYMODE)
        publishTrainMessage(cppInterface, result.state, result.terminated, result.reward);
      else
        publishNonTrainMessage(cppInterface, result.state, result.terminated);
    }
  }

  //called when self-play agent calls for a reset
  public void reset2(boolean reset) {
    if (reset && in_termination && PLAYMODE) {
      reset1(reset);
    }
  }

  public Result reset() {
    System.out.println("RESETTING");
    ++this.episodeCounter;
    this.stepCounter = 0;
    return this.setup();
  }

  //if the agent is not trained the environment has to provide the
  //initial messages (NOT NEEDED)
  //public void sendFirst(){
  ////try{
  ////Thread.sleep(3000);
  ////} catch (InterruptedException ex) {}
  //currentVehicle = 0;
  //reset1(true);
  //}

  //starts a new simulation and initializes all variables
  private Result setup() {
    if (simulator != null) {
      simulator.destroy();
      simulator = null;
    }
    if (viz != null) {
      viz.clearRenderer();
    }

    // Randomize Scenario
    try {
      RandomizationProperties randomizationProperties  = RandomRandomizationPropertiesPicker.pickRandomizationProperties(config.randomization);
      RandomizationStrategy strategy = randomizationProperties.build(config, "");
      config.cars = strategy.randomizeCars(config.cars);
      config.map_name = strategy.randomizeMapName(config.map_name);
      config.max_duration = strategy.randomizeMaxDuration(config.max_duration);
      config.tick_duration = strategy.randomizeTickDuration(config.tick_duration);
      config.modules = strategy.randomizeModules(config.modules);
      config.preprocessor = strategy.randomizePreprocessor(config.preprocessor);
      config.rewardFunction = strategy.randomizeRewardFunction(config.rewardFunction);
    }
    catch (Exception e) {
      e.printStackTrace();
    }

    world = new OsmToWorldLoader(map).getWorld();
    pathfinding = new PathfindingImpl(world);
    simulator = config.build(world, pathfinding, map);
    vehiclesArray = simulator.getVehicles().toArray(Vehicle[]::new);
    if (viz != null) {
      viz.simTime = simulationTime;
      viz.setup(world, pathfinding);
    }

    if (config.tick_duration.toMillis() <= 100l) {
      update_iterations = (int) (100l / config.tick_duration.toMillis());
    }

    //parse all required components
    autopilots = new RLAutopilot[vehiclesArray.length];
    for (int i = 0; i < autopilots.length; i++) {
      autopilots[i] = (RLAutopilot) vehiclesArray[i].eesystem.getComponent("RLAutopilot").get();
    }

    navigations = new Navigation[autopilots.length];
    for (int i = 0; i < navigations.length; i++) {
      navigations[i] = (Navigation) vehiclesArray[i].eesystem.getComponent("Navigation").get();
    }

    truePositions = new TruePosition[autopilots.length];
    trueVelocity = new TrueVelocity[autopilots.length];
    trueCompass = new TrueCompass[autopilots.length];
    lidars = new Lidar[autopilots.length];
    speedLimitServices = new SpeedLimitService[autopilots.length];
    for (int i = 0; i < truePositions.length; i++) {
      truePositions[i] = (TruePosition) vehiclesArray[i].physicalValues.getPhysicalValue("true_position");
      trueVelocity[i] = (TrueVelocity) vehiclesArray[i].physicalValues.getPhysicalValue("true_velocity");
      trueCompass[i] = (TrueCompass) vehiclesArray[i].physicalValues.getPhysicalValue("true_compass");
      if (vehiclesArray[i].eesystem.getComponent("Lidar").isPresent()) {
        lidars[i] = (Lidar) vehiclesArray[i].eesystem.getComponent("Lidar").get();
        lidar_offset = lidars[i].getMessageLength();
      }
      if (vehiclesArray[i].eesystem.getComponent("SpeedLimit").isPresent()) {
        speedLimitServices[i] = (SpeedLimitService) vehiclesArray[i].eesystem.getComponent("SpeedLimit").get();
        speed_limit_offset = 1;
      }
    }

    stateLength = DEFAULT_STATE_LENGTH + lidar_offset + speed_limit_offset;
    vehicleStates = new float[autopilots.length][stateLength];

    //update simulation until all values are assigned
    while (anyStateNull()) {
      TimeUpdate tu = new TimeUpdate(simulationTime, config.tick_duration);
      simulator.update(tu);
      simulationTime = tu.newTime;
    }
    activeVehicle = 0;
    receiveActions = true;

    if (selfPlay_mode == "afterEpisode") { // afterEpisode option can be selected via CLI
      trainedVehicle = (trainedVehicle + 1) % vehiclesArray.length; //pick the next vehicle as the trained vehicle
    }
    updateStatePackets();
    float[] simState = getState();

    rewardFunction = config.rewardFunction.map(props -> props.build(vehiclesArray, config.tick_duration)).orElse(new DefaultRewardFunctionProperties().build(vehiclesArray, config.tick_duration));

    float init_reward = rewardFunction.getReward(0);
    boolean simTermination = this.getSimTermination();

    //check if all vehicles found a path, if not, restart simulation
    for (int i = 0; i < navigations.length; i++) {
      if (navigations[i].getCurrentPath().get().getLength() == 0)
        return setup();
    }
    setupComplete = true;

    return new Result(init_reward, simState, simTermination);
  }

  private Result step(float[] action) {
    stepCounter++;
    //self-play and training mode
    if (distributed && PLAYMODE == false && !miniStep) {
      int listCounter = 0; //makes sure that right action out of the decentralizedActionsList is selected
      for (int i = 0; i < autopilots.length; i++) {

        if (i == trainedVehicle) {
          setAction(action); //set action for the trained vehicle
        }
        else if (i != trainedVehicle && decentralizedActionsList != null && decentralizedActionsList.size() != 0) {
          if (decentralizedActionsList.size() >= i) { //set self-play actions
            setDistributedAction(decentralizedActionsList.get(listCounter), i);
            listCounter += 1;
          }
          else if (decentralizedActionsList.size() < i && decentralizedActionsList.size() >= 1) {
            setDistributedAction(decentralizedActionsList.get(0), i);
          }
        }
      }
      listCounter = 0;

    }
    //set action for all vehicles in self-play execution mode
    else if (distributed && PLAYMODE == true && !miniStep) {
      if (decentralizedActionsList != null && decentralizedActionsList.size() != 0) {
        for (int i = 0; i <= decentralizedActionsList.size() - 1; i++) {
          setDistributedAction(decentralizedActionsList.get(i), i);

        }
      }
    }
    //set action for single-vehicle and centralized case
    else {
      setAction(action);
    }

    if (simulator == null)
      return new Result();

    if (!miniStep) {
      for (int i = 0; i < update_iterations; i++) {
        if (simulator != null) {
          TimeUpdate tu = new TimeUpdate(simulationTime, config.tick_duration);
          simulator.update(tu);
          simulationTime = tu.newTime;
        }
      }
    }
    else {
      for (int i = 0; i < update_iterations; i++) {
        if (simulator != null) {
          TimeUpdate tu = new TimeUpdate(simulationTime, config.tick_duration.dividedBy(vehiclesArray.length));
          simulator.update(tu);
          simulationTime = tu.newTime;
        }
      }
    }

    updateStatePackets();
    float[] simState = getState();
    float step_reward;

    if (!distributed) {
      step_reward = rewardFunction.getReward(stepCounter);
    }
    else if (miniStep) {
      step_reward = rewardFunction.getRewardForVehicle(activeVehicle, stepCounter);
    }
    else {
      step_reward = rewardFunction.getRewardForVehicle(trainedVehicle, stepCounter); //calculate reward only for trained vehicle in self-play mode
    }

    boolean simTermination = this.getSimTermination();
    activeVehicle = (activeVehicle + 1) % vehiclesArray.length;

    if (distributed & !miniStep) {

      while (decentralizedActionsList.size() > 0) { //clear self-play action list for next step
        decentralizedActionsList.remove(0);
      }
    }
    currentVehicle = 0;

    //afterStep option is selected in CLI
    if (selfPlay_mode.equals("afterStep")) {
      trainedVehicle = (trainedVehicle + 1) % vehiclesArray.length; //pick next vehicle as the trainedVehicle
    }

    if (viz != null) {
      try {
        viz.simTime = simulationTime;
        viz.redraw();
      }
      catch (ConcurrentModificationException ignore) {
      }
    }
    if (simTermination && config.collision_mode.equals("LOG_COLLISIONS")) {
      CollisionLogWriter.addCollisions(simulator.collisionHistory, config.start_time, true, !PLAYMODE, episodeCounter);
      VelocityLogWriter.addVelocities(simulator.velocityHistory, config.start_time, true, !PLAYMODE, episodeCounter);
    }
    done = true;
    receiveActions = true;
    return new Result(step_reward, simState, simTermination);
  }

  private void updateStatePackets() {
    for (int i = 0; i < autopilots.length; i++) {
      autopilots[i].updateStatePacket();
    }
  }

  private void wait_done() {
    while (!done)
      ;
    done = false;
    return;
  }

  private boolean getSimTermination() {
    return simulator.status(stepCounter) != TaskStatus.RUNNING;
  }

  //return the state array that is published on the state topic
  private float[] getState() {
    int vehicleCount = autopilots.length;
    int statePacketLength = autopilots[0].getStatePacket().length;
    updateVehicleStates();

    float[] result;
    if (!distributed) {
      result = new float[vehicleCount * stateLength];

      for (int i = 0; i < vehicleCount; i++) {
        for (int j = 0; j < stateLength; j++) {
          result[i * stateLength + j] = vehicleStates[i][j];
        }
      }
    }
    else {
      int currentVehicle = (miniStep) ? activeVehicle : trainedVehicle;
      result = getPreprocessedStates(currentVehicle);
    }
    return result;
  }

  //method to return the state to the self-play vehicle
  private float[] getDistributedState(int index) {
    int stateLength = autopilots[0].state.length;
    updateVehicleStates();
    float[] result;
    result = getPreprocessedStates(index);
    return result;
  }

  /**
   * Returns the current preprocessed state
   *
   * @param currentVehicleIndex The index of the current vehicle
   * @return A float[] containing the preprocessed state
   */
  private float[] getPreprocessedStates(int currentVehicleIndex) {
    int vehicleCount = autopilots.length;
    int statePacketLength = autopilots[0].getStatePacket().length;
    // Reshape the vehicle states
    float[] activeVehicleState = new float[vehicleStates[currentVehicleIndex].length];
    for (int i = 0; i < activeVehicleState.length; i++) {
      activeVehicleState[i] = vehicleStates[currentVehicleIndex][i];
    }
    float[][] otherVehicleStates = new float[vehicleCount - 1][statePacketLength];
    for (int i = 1; i < vehicleCount; i++) {
      float[] statePacket = autopilots[(currentVehicleIndex + i) % vehicleCount].getStatePacket();
      for (int j = 0; j < statePacketLength; j++) {
        otherVehicleStates[i - 1][j] = statePacket[j];
      }
    }
    // Preprocess the states and return the result
    float[] result = simulator.preprocessor.preprocessState(activeVehicleState, otherVehicleStates, statePacketLength);
    return result;
  }

  //set the combined action from all vehicles
  private void setAction(float[] action) {

    int vehicleCount = autopilots.length;
    int actionLength = action.length / vehicleCount; //assume that every vehicle has same action space

    if (!distributed) { //single vehicle or centralized
      for (int i = 0; i < vehicleCount; i++) {
        float[] result = new float[actionLength];
        for (int j = 0; j < actionLength; j++) {
          result[j] = action[i * actionLength + j];
        }
        autopilots[i].action = result;
      }
    }
    else if (miniStep) {

      autopilots[activeVehicle].action = action; //mini-step approach
    }
    else {

      autopilots[trainedVehicle].action = action; //self-play approach
    }
    return;
  }

  //set action of the self play agent
  private void setDistributedAction(float[] action, int index) {
    autopilots[index].action = action;
  }

  private boolean anyStateNull() {
    for (int i = 0; i < autopilots.length; i++) {
      if (autopilots[i].state == null)
        return true;
    }
    return false;
  }

  //assign random start and target coordinates for all vehicles
  //addtional orientation allignment with first segment

  public Simulator getSim() {
    return simulator;
  }

  //update the array containing the states of all vehicles
  public void updateVehicleStates() {
    for (int i = 0; i < autopilots.length; i++) {
      for (int j = 0; j < stateLength; j++) {
        vehicleStates[i][j] = 0.f;
      }
      int trajectoryLength = navigations[i].getCurrentTrajSize();
      for (int j = 0; j < trajectoryLength; j++) {
        vehicleStates[i][j] = (float) navigations[i].getCurrentTraj()[j].at(0);
        vehicleStates[i][10 + j] = (float) navigations[i].getCurrentTraj()[j].at(1);
      }
      vehicleStates[i][20] = (float) trajectoryLength;
      vehicleStates[i][21] = (float) ((Vec2) truePositions[i].get()).at(0);
      vehicleStates[i][22] = (float) ((Vec2) truePositions[i].get()).at(1);
      vehicleStates[i][23] = (float) ((Double) trueCompass[i].get()).doubleValue();
      vehicleStates[i][24] = (float) ((Double) trueVelocity[i].get()).doubleValue();
      if (speed_limit_offset > 0) {
        vehicleStates[i][25] = (float) speedLimitServices[i].getSpeedLimit(0);
      }
      if (lidar_offset > 0) {
        for (int j = 0; j < lidar_offset; j++) {
          vehicleStates[i][DEFAULT_STATE_LENGTH + speed_limit_offset + j] = (float) lidars[i].getLidarValue(j);
        }
      }
    }
  }

  // set reinforcement learning settings
  public void setSettings(boolean distributed, boolean randomize, boolean play, boolean miniStep, String selfPlay_mode) {
    this.distributed = distributed;
    this.randomize = randomize;
    this.PLAYMODE = play;
    this.miniStep = miniStep;
    this.selfPlay_mode = selfPlay_mode;
  }

  public native void startros();

  public native void publishNonTrainMessage(long interf, float[] state, boolean terminal);

  public native void publishTrainMessage(long interf, float[] state, boolean terminal, float reward);

  public void start() {
    startros();
  }

  //store cpp ROSInterface object
  public void setCppInterface(long cppInterface) {
    this.cppInterface = cppInterface;
  }

  private class Result {
    float reward = 0f;
    float[] state = null;
    boolean terminated = true;

    public Result(float reward, float[] state, boolean terminated) {
      this.reward = reward;
      this.state = state;
      this.terminated = terminated;
    }

    public Result() {
    }
  }

  //public void checkPlay(){
  //if(PLAYMODE) sendFirst();
  //}
}