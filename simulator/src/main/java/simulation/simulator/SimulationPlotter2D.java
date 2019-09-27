/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.simulator;

import org.apache.commons.math3.linear.RealVector;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopExecutable;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopNotifiable;
import simulation.util.Plotter2D;
import simulation.vehicle.MassPointPhysicalVehicle;
import simulation.vehicle.ModelicaPhysicalVehicle;

import java.time.Duration;
import java.time.Instant;
import java.util.LinkedList;
import java.util.List;

/**
 * Class that collects vehicle logging data stores them and sends them to to Plotter 2D for  plotting
 */

public class SimulationPlotter2D extends SimulationLoopNotifiable {

    // Lists to store the data separately for plotting later
    private List<Instant> simulationTimePoints = new LinkedList<>();
    private List<List<Double>> wheelRotationRates = new LinkedList<>();
    private List<RealVector> vehiclePosition = new LinkedList<>();
    private List<RealVector> vehicleVelocity= new LinkedList<>();
    private List<List<RealVector>> wheelPositions = new LinkedList<>();

    /**
     * This function retrieves data of the vehicle at every instant and stores it
     * for plotting by the Plotter2D function. To be executed for every iteration in the simulation
     * @param simulationObject PhysicalVehicle object that provides position and velocity data of the vehicle
     * @param totalTime Total simulation time
     * @param timeDiffms Delta simulation time
     */
    @Override
    public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime, Duration timeDiffms) {

        // Check if the argument is of type PhysicalVehicle
        if (simulationObject instanceof ModelicaPhysicalVehicle) {
            // Convert the argument to PhysicalVehicle type
            ModelicaPhysicalVehicle plottingVehicle = ((ModelicaPhysicalVehicle) simulationObject);

            // Get current position and velocity of the vehicle
            vehiclePosition.add(plottingVehicle.getPosition());
            vehicleVelocity.add(plottingVehicle.getVelocity());
            List<Double> wheelsBuffer1 = new LinkedList<>();
            wheelsBuffer1.add(plottingVehicle.getVDM().getValue("omega_wheel_1"));
            wheelsBuffer1.add(plottingVehicle.getVDM().getValue("omega_wheel_2"));
            wheelsBuffer1.add(plottingVehicle.getVDM().getValue("omega_wheel_3"));
            wheelsBuffer1.add(plottingVehicle.getVDM().getValue("omega_wheel_4"));
            wheelRotationRates.add(wheelsBuffer1);
            List<RealVector> wheelsBuffer2 = new LinkedList<>();
            wheelsBuffer2.add(plottingVehicle.getFrontLeftWheelGeometryPosition());
            wheelsBuffer2.add(plottingVehicle.getFrontRightWheelGeometryPosition());
            wheelsBuffer2.add(plottingVehicle.getBackLeftWheelGeometryPosition());
            wheelsBuffer2.add(plottingVehicle.getBackRightWheelGeometryPosition());
            wheelPositions.add(wheelsBuffer2);

            // Store current simulation time
            simulationTimePoints.add(Simulator.getSharedInstance().getSimulationTime());
        }else if(simulationObject instanceof MassPointPhysicalVehicle){
            // Convert the argument to PhysicalVehicle type
            MassPointPhysicalVehicle plottingVehicle = ((MassPointPhysicalVehicle) simulationObject);

            // Get current position and velocity of the vehicle
            vehiclePosition.add(plottingVehicle.getPosition());
            vehicleVelocity.add(plottingVehicle.getVelocity());
            List<Double> wheelsBuffer1 = new LinkedList<>();
            wheelsBuffer1.add(new Double(0.0));
            wheelsBuffer1.add(new Double(0.0));
            wheelsBuffer1.add(new Double(0.0));
            wheelsBuffer1.add(new Double(0.0));
            wheelRotationRates.add(wheelsBuffer1);
            List<RealVector> wheelsBuffer2 = new LinkedList<>();
            wheelsBuffer2.add(plottingVehicle.getFrontLeftWheelGeometryPosition());
            wheelsBuffer2.add(plottingVehicle.getFrontRightWheelGeometryPosition());
            wheelsBuffer2.add(plottingVehicle.getBackLeftWheelGeometryPosition());
            wheelsBuffer2.add(plottingVehicle.getBackRightWheelGeometryPosition());
            wheelPositions.add(wheelsBuffer2);

            // Store current simulation time
            simulationTimePoints.add(Simulator.getSharedInstance().getSimulationTime());
        }
    }


    /**
     * Function that creates Plots using instances of the Plotter 2D with arguments consisting of the stored data from
     * the simulation
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time in milliseconds
     **/
    @Override
    public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, Instant totalTime) {
        Plotter2D.plot(vehiclePosition, vehicleVelocity, wheelRotationRates, simulationTimePoints);
    }

    @Override
    public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime, Duration deltaTime) {}

    @Override
    public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime, Duration deltaTime) {}

    @Override
    public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime, Duration deltaTime) {}

    @Override
    public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {}
}
