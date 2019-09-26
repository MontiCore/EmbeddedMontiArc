/**
 *
 * /* (c) https://github.com/MontiCore/monticore */
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package simulation.runner;

import commons.controller.commons.BusEntry;
import commons.controller.commons.Vertex;
import commons.simulation.Sensor;
import commons.simulation.SimulationLoopExecutable;
import commons.simulation.SimulationLoopNotifiable;
import databus.DataBus;
import functionBlock.ConnectionEntry;
import functionBlock.FunctionBlock;
import navigationBlock.components.FindPath;
import sensors.StaticPlannedTrajectoryXSensor;
import sensors.StaticPlannedTrajectoryYSensor;
import sensors.util.SensorUtil;
import simulation.environment.WorldModel;
import simulation.environment.osm.ParserSettings;
import simulation.environment.weather.WeatherSettings;
import simulation.simulator.Simulator;
import simulation.vehicle.*;
import structures.Graph;

import rwth.server.simulation.adapters.AutoPilot.AutopilotAdapterAsFunctionBlock;
import rwth.server.simulation.adapters.util.RMIClient;

import java.util.*;

public class Runner {
    private  String mapPath;
    private  long sourceOsmNode;
    private  long targetOsmNode;
    private  boolean useModelicaVehicle;

    // hostname and port of RMIServer
    private String autopilotHost;
    private int autopilotPort;

    public Runner(String mapPath, long sourceOsmNode, long targetOsmNode, boolean useModelicaVehicle, String autopilotHost, int autopilotPort) {
        this.mapPath = mapPath;
        this.sourceOsmNode = sourceOsmNode;
        this.targetOsmNode = targetOsmNode;
        this.useModelicaVehicle = useModelicaVehicle;
        this.autopilotHost = autopilotHost;
        this.autopilotPort = autopilotPort;
    }

    public double run(){
        try {
            WorldModel.init(
                    new ParserSettings(mapPath, ParserSettings.ZCoordinates.ALLZERO),
                    new WeatherSettings()
            );
        } catch (Exception e) {
            e.printStackTrace();
        }

        List<Vertex> trajectory = computeTrajectory(sourceOsmNode, targetOsmNode);

        RMIClient rmiClient = new RMIClient(autopilotHost, autopilotPort, "");
        AutopilotAdapterAsFunctionBlock mainBlockAdapter = new AutopilotAdapterAsFunctionBlock(rmiClient);

        // init model
        PhysicalVehicle physicalVehicle;
        if(useModelicaVehicle){
            ModelicaPhysicalVehicleBuilder vehicleBuilder = new ModelicaPhysicalVehicleBuilder();
            vehicleBuilder.setControllerBus(Optional.of(new DataBus()));
            vehicleBuilder.setController(Optional.of(mainBlockAdapter));
            physicalVehicle = vehicleBuilder.buildPhysicalVehicle();
        }else{
            MassPointPhysicalVehicleBuilder vehicleBuilder = new MassPointPhysicalVehicleBuilder();
            vehicleBuilder.setControllerBus(Optional.of(new DataBus()));
            vehicleBuilder.setController(Optional.of(mainBlockAdapter));
            physicalVehicle = vehicleBuilder.buildPhysicalVehicle();
        }

        physicalVehicle.getSimulationVehicle().setStatusLogger(new RandomStatusLogger());
        SensorUtil.sensorAdder(physicalVehicle);

        mainBlockAdapter.setVehicle(physicalVehicle);
        rmiClient.setVehicleId(physicalVehicle.getId());

        long simulationDuration = 15000;
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.registerAndPutObject(physicalVehicle,
                trajectory.get(0).getPosition().toArray()[0],
                trajectory.get(0).getPosition().toArray()[1],
                getRotation(trajectory)
        );

        //  setup its trajectory
        Map<String, List<Double>> trajectoryCoordinates = getTrajectoryCoordinates(trajectory);
        Vehicle simVehicle = physicalVehicle.getSimulationVehicle();
        simVehicle.addSensor(new StaticPlannedTrajectoryXSensor(trajectoryCoordinates.get("x")));
        simVehicle.addSensor(new StaticPlannedTrajectoryYSensor(trajectoryCoordinates.get("y")));

        SimulationLoopNotifiable observer = new Observer();
        sim.registerLoopObserver(observer);
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(simulationDuration);
        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        return ((Observer) observer).distanceToTarget;
    }

    private static List<Vertex> computeTrajectory(long sourceOsmID, long targetOsmID){
        Graph graph = new Graph(WorldModel.getInstance().getControllerMap().getAdjacencies(), true);
        FunctionBlock findPath = new FindPath();
        Map<String, Object> inputs = new HashMap<String, Object>();
        inputs.put(ConnectionEntry.FIND_PATH_graph.toString(), graph);
        inputs.put(ConnectionEntry.FIND_PATH_start_vertex.toString(), getVertexForOsmId(graph, sourceOsmID));
        inputs.put(ConnectionEntry.FIND_PATH_target_vertex.toString(),getVertexForOsmId(graph, targetOsmID));
        findPath.setInputs(inputs);
        findPath.execute();

        List<Vertex> result = (List<Vertex>) findPath.getOutputs().get(ConnectionEntry.FIND_PATH_path.toString());
        for (Vertex v : result) {
            System.out.printf("%s, %s\n", v.getOsmId(), Arrays.toString(v.getPosition().toArray()));
        }
        return result;
    }

    public static Vertex getVertexForOsmId(Graph graph, long osmId) {
        for (Vertex v : graph.getVertices())
            if (v.getOsmId().longValue() == osmId) return v;
        return null;
    }

    public static Map<String, List<Double>> getTrajectoryCoordinates(List<Vertex> trajectory) {
        Map<String, List<Double>> result = new HashMap<String, List<Double>>();

        List<Double> x = new ArrayList<Double>();
        List<Double> y = new ArrayList<Double>();
        for (Vertex v : trajectory) {
            double[] pos = v.getPosition().toArray();
            x.add(pos[0]);
            y.add(pos[1]);
        }
        result.put("x", x);
        result.put("y", y);

        return result;
    }

    public static double getRotation(List<Vertex> trajectory) {
        if (trajectory.size() < 2) return 0d;
        //compare first two vectors, to get correct start rotation
        double[] v1pos = trajectory.get(0).getPosition().toArray();
        double[] v2pos = trajectory.get(1).getPosition().toArray();

        //slightly change the rotation from perfect straight line,
        //otherwise the simulator calculates strange starting trajectory
        return Math.atan2(v1pos[0] - v2pos[0], v2pos[1] - v1pos[1]) -0.35d;
    }

    private static class Observer implements SimulationLoopNotifiable {
        double distanceToTarget = 0;

        @Override
        public void willExecuteLoop(List<SimulationLoopExecutable> list, long l, long l1) {

        }

        @Override
        public void didExecuteLoop(List<SimulationLoopExecutable> list, long l, long l1) {

        }

        @Override
        public void willExecuteLoopForObject(SimulationLoopExecutable simulationLoopExecutable, long l, long l1) {

        }

        @Override
        public void didExecuteLoopForObject(SimulationLoopExecutable simulationLoopExecutable, long l, long l1) {
            PhysicalVehicle vehicle = (PhysicalVehicle) simulationLoopExecutable;
            double[] position = vehicle.getPosition().toArray();
            String id = String.valueOf((int) ((PhysicalVehicle) simulationLoopExecutable).getId());

            Sensor sensorX = ((PhysicalVehicle) simulationLoopExecutable).getSimulationVehicle().getSensorByType(BusEntry.PLANNED_TRAJECTORY_X).get();
            Sensor sensorY = ((PhysicalVehicle) simulationLoopExecutable).getSimulationVehicle().getSensorByType(BusEntry.PLANNED_TRAJECTORY_Y).get();
            List<Double> trajectoryX = (List<Double>)sensorX.getValue();
            List<Double> trajectoryY = (List<Double>)sensorY.getValue();
            double targetX = trajectoryX.get(trajectoryX.size() - 1);
            double targetY = trajectoryY.get(trajectoryY.size() - 1);
            double dist = Math.sqrt(Math.pow(targetX - position[0], 2) + Math.pow(targetY - position[1], 2));
            distanceToTarget = dist;

            System.out.printf("id: %s, position: [%.8f, %.8f, %.6f], target: (%.8f, %.8f), dist: %.8f, engine: %.4f, brake: %.4f, steering: %.4f\n",
                    id, position[0], position[1], position[2], targetX, targetY, dist,
                    vehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR).getActuatorValueCurrent(),
                    vehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).getActuatorValueCurrent(),
                    vehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING).getActuatorValueCurrent()
            );
        }

        @Override
        public void simulationStarted(List<SimulationLoopExecutable> list) {

        }

        @Override
        public void simulationStopped(List<SimulationLoopExecutable> list, long l) {

        }
    }
}
