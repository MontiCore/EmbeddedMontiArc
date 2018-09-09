package simulation.util;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.axis.NumberAxis;
import java.awt.Color;
import java.io.File;
import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.linear.RealVector;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.ui.ApplicationFrame;
import org.jfree.chart.plot.XYPlot;

import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;

/**
 * Class that Plots the a 2D Graph for two datasets
 */
public class Plotter2D extends ApplicationFrame
{
    public final static int PLOTTER_OUTPUT_POSITION_XY = 1;
    public final static int PLOTTER_OUTPUT_VELOCITY = 2;
    public final static int PLOTTER_OUTPUT_Z = 3;

    /* A circle shape to be used by the plotting renderer */
    java.awt.geom.Ellipse2D.Double shape = new java.awt.geom.Ellipse2D.Double(-2.0, -2.0,   2.0, 2.0);

    /**
     * Constructor of the Plotter 2D object which responsible of creating
     * the charts for the different data sets related to the vehicle's
     * movement.
     * It returns a Plotter 2D object that contains graphs of the data set
     * specified by the "OutputType" string.
     * The charts could be displayed using specific functions of the JFree library
     * " pack()" and  "setVisible()"
     * @param  wheelPositions The relative positions of the wheels respective center of mass
     * @param  vehiclePosition Relative position of the center of mass of the vehicle
     * @param  vehicleVelocity Velocity of the vehicle's center of mass
     * @param  simulationTimePoints Discrete time vector duration of the simulation
     * @param  outputType an integer to define which data set to be returned for display
     */

    public Plotter2D(List<List<RealVector>> wheelPositions, List<RealVector> vehiclePosition, List<RealVector> vehicleVelocity, List<Long> simulationTimePoints, int outputType) {
        super("Vehicle Simulation Charts");

        // Selecting which data charts to return for display
        switch (outputType) {
            case PLOTTER_OUTPUT_POSITION_XY:
                // Creating charts for both position and velocity based on the provided data sets
                JFreeChart PositionChart = vehiclePositionChart(wheelPositions, vehiclePosition, simulationTimePoints);

                final ChartPanel PositionPanel = new ChartPanel(PositionChart, true, true, true, true, true);

                // Chart dimensions
                PositionPanel.setPreferredSize(new java.awt.Dimension(800, 600));

                try {
                    ChartUtilities.saveChartAsPNG(new File("position.png"), PositionChart, 600, 800);
                }catch (Exception e){
                    e.printStackTrace();
                }
                setContentPane(PositionPanel);
                break;
            case PLOTTER_OUTPUT_VELOCITY:
                // Creating charts for both position and velocity based on the provided data sets
                JFreeChart VelocityChart = vehicleVelocityChart(vehicleVelocity, simulationTimePoints);
                final ChartPanel VelocityPanel = new ChartPanel(VelocityChart, true, true, true, true, true);

                // Chart dimensions
                VelocityPanel.setPreferredSize(new java.awt.Dimension(800, 600));

                try {
                    ChartUtilities.saveChartAsPNG(new File("velocity.png"), VelocityChart, 600, 800);
                }catch(Exception e){
                    e.printStackTrace();
                }

                setContentPane(VelocityPanel);
                break;
            case PLOTTER_OUTPUT_Z:
                // Creating charts for both position and velocity based on the provided data sets
                JFreeChart zPositionChart = zAxisChart(wheelPositions, simulationTimePoints);
                final ChartPanel zAxisPanel = new ChartPanel(zPositionChart, true, true, true, true, true);

                // Chart dimensions
                zAxisPanel.setPreferredSize(new java.awt.Dimension(800, 600));

                try {
                    ChartUtilities.saveChartAsPNG(new File("z.png"), zPositionChart, 600, 800);
                }catch(Exception e){
                    e.printStackTrace();
                }

                setContentPane(zAxisPanel);
                break;
            default:
                Log.warning("Plotter: Incorrect plotter output type");
                break;
        }
    }


    /**
     * Function that creates the Vehicle position 2D plot charts
     * for both the position of the center of mass of the vehicle
     * and the four wheels centers of mass.
     *
     * @param wheelPositions Positions of the wheels of the vehicle
     * @param vehiclePosition Position of the center of mass of the vehicle
     * @param  simTime Discrete time vector of duration of the simulation
     * @return Returns a JFree chart object for which a panel needs to be created for display
     */

    private JFreeChart vehiclePositionChart(List<List<RealVector>> wheelPositions, List<RealVector> vehiclePosition, List<Long> simTime) {
        // Set axis names
        final NumberAxis xAxis = new NumberAxis("X Axis");
        final ValueAxis yAxis = new NumberAxis("Y axis");

        // Create position data sets for vehicle and wheel position
        final XYDataset wheelDataSet = wheelDataSet(wheelPositions);
        final XYDataset vehiclePositionDataSet = vehiclePositionDataSet(vehiclePosition, simTime);

        // Creating and customizing renderer setting
        final XYLineAndShapeRenderer VehiclePositionRenderer = new XYLineAndShapeRenderer();
        VehiclePositionRenderer.setBaseShapesFilled(false);
        VehiclePositionRenderer.setSeriesPaint( 0 , Color.BLUE );
        VehiclePositionRenderer.setSeriesShape(0, shape);
        VehiclePositionRenderer.setSeriesPaint( 1 , Color. RED );
        VehiclePositionRenderer.setSeriesShape(1, shape);
        VehiclePositionRenderer.setSeriesPaint( 2 , Color.GREEN );
        VehiclePositionRenderer.setSeriesShape(2, shape);
        VehiclePositionRenderer.setSeriesPaint( 3 , Color.YELLOW );
        VehiclePositionRenderer.setSeriesShape(3, shape);
        VehiclePositionRenderer.setSeriesPaint( 4 , Color.BLACK );

        VehiclePositionRenderer.setSeriesLinesVisible(0, false);
        VehiclePositionRenderer.setSeriesLinesVisible(1, false);
        VehiclePositionRenderer.setSeriesLinesVisible(2, false);
        VehiclePositionRenderer.setSeriesLinesVisible(3, false);
        VehiclePositionRenderer.setSeriesLinesVisible(4, false);

        // Creating Position Plot object for wheels and adding the vehicle Center of mass
        XYPlot positionPlot = new XYPlot(wheelDataSet, xAxis, yAxis, VehiclePositionRenderer);
        positionPlot.setDataset(1, vehiclePositionDataSet);

        return new JFreeChart("Vehicle Simulation : Vehicle Position Chart", JFreeChart.DEFAULT_TITLE_FONT, positionPlot, true);
    }

    /**
     * Function that creates the Vehicle position 2D plot charts
     * for both the position of the center of mass of the vehicle
     * and the four wheels centers of mass.
     *
     * @param WheelsPosition Relative position of the center of mass of the vehicle
     * @param  SimTime Discrete time vector of duration of the simulation
     * @return Returns a JFree chart object for which a panel needs to be created for display
     */

    private JFreeChart zAxisChart(List<List<RealVector>> WheelsPosition,  List<Long> SimTime ) {
        // Set axis names
        final NumberAxis xAxis = new NumberAxis("Time ");
        final ValueAxis yAxis = new NumberAxis("Z ");

        // Creating Position Dataset for vehicle and wheels center of mass
        final XYDataset VehicleZAxis = zAxisDataSet(WheelsPosition, SimTime);

        // Creating and customizing renderer setting
        final XYLineAndShapeRenderer zPositionRenderer = new XYLineAndShapeRenderer();

        zPositionRenderer.setBaseShapesFilled(false);
        zPositionRenderer.setSeriesPaint( 0 , Color.BLUE );
        zPositionRenderer.setSeriesShape(0, shape);
        zPositionRenderer.setSeriesPaint( 1 , Color. RED );
        zPositionRenderer.setSeriesShape(1, shape);
        zPositionRenderer.setSeriesPaint( 2 , Color.GREEN );
        zPositionRenderer.setSeriesShape(2, shape);
        zPositionRenderer.setSeriesPaint( 3 , Color.YELLOW );
        zPositionRenderer.setSeriesShape(3, shape);
        zPositionRenderer.setSeriesPaint( 4 , Color.BLACK );

        zPositionRenderer.setSeriesLinesVisible(0, false);
        zPositionRenderer.setSeriesLinesVisible(1, false);
        zPositionRenderer.setSeriesLinesVisible(2, false);
        zPositionRenderer.setSeriesLinesVisible(3, false);
        zPositionRenderer.setSeriesLinesVisible(4, false);

        // Creating Position Plot object for wheels and adding the vehicle Center of mass
        XYPlot zAxisPlot  = new XYPlot(VehicleZAxis, xAxis, yAxis, zPositionRenderer);

        return new JFreeChart("Vehicle Simulation : Vehicle zAxis Position Chart", JFreeChart.DEFAULT_TITLE_FONT, zAxisPlot, true);

    }

    /**
     * Function that creates the Vehilcle Velocity 2D plot charts
     * for the center of mass of the vehicle
     *
     * @param vehicleVelocity Relative Velocity of the center of mass of the vehicle
     * @param  simTime Discrete time vector of duration of the simulation
     * @return Returns a JFree chart object for which a panel needs to be created for display
     */
    private JFreeChart vehicleVelocityChart(List<RealVector> vehicleVelocity, List<Long> simTime ) {
        // Set axis names
        final NumberAxis xAxis = new NumberAxis("Simulation Time (ms)");
        final ValueAxis yAxis = new NumberAxis("Vehicle Velocity");

        // Creating Velocity Data set
        final XYDataset velocity = vehicleVelocityDataSet(vehicleVelocity, simTime);

        // Creating and customizing renderer setting
        final XYLineAndShapeRenderer VelocityPlotRenderer = new XYLineAndShapeRenderer();
        VelocityPlotRenderer.setBaseShapesFilled(false);
        VelocityPlotRenderer.setSeriesPaint(0, Color.BLUE );
        VelocityPlotRenderer.setSeriesShape(0, shape);

        VelocityPlotRenderer.setSeriesLinesVisible(0, false);

        // Creating Velocity Plot object for vehicle
        XYPlot velocityPlot = new XYPlot(velocity, xAxis, yAxis, VelocityPlotRenderer);

        return new JFreeChart("Vehicle Simulation : Velocity Chart", JFreeChart.DEFAULT_TITLE_FONT, velocityPlot, true);
    }

    /**
     * Function that creates XY datasets for every wheel center of mass
     *
     * @param wheelPositions a vector containing the XYZ coordinates of the wheels obtained during the simulation
     * @return Returns an XY series representing the relative position of every wheel in the plane for plotting
     */
    private XYDataset wheelDataSet(List<List<RealVector>> wheelPositions) {

        XYSeries wheelOne = new XYSeries("WheelOne", false, true);
        XYSeries wheelTwo = new XYSeries("WheelTwo", false, true);
        XYSeries wheelThree = new XYSeries("WheelThree", false, true);
        XYSeries wheelFour = new XYSeries("WheelFour", false, true);

        // Retrieving XY coordinates of each of the 4 wheels of the vehicle
        for(int i=0; i<wheelPositions.size(); i++){

            // storing XY coordinates in their respective series
            wheelOne.add(wheelPositions.get(i).get(0).getEntry(0), wheelPositions.get(i).get(0).getEntry(1));
            wheelTwo.add(wheelPositions.get(i).get(1).getEntry(0), wheelPositions.get(i).get(1).getEntry(1));
            wheelThree.add(wheelPositions.get(i).get(2).getEntry(0), wheelPositions.get(i).get(2).getEntry(1));
            wheelFour.add(wheelPositions.get(i).get(3).getEntry(0), wheelPositions.get(i).get(3).getEntry(1));
        }

        // Create a parallel series collection to store data of the 4 wheels
        XYSeriesCollection WheelsDataSet = new XYSeriesCollection();

        WheelsDataSet.addSeries(wheelOne);
        WheelsDataSet.addSeries(wheelTwo);
        WheelsDataSet.addSeries(wheelThree);
        WheelsDataSet.addSeries(wheelFour);

        return WheelsDataSet;
    }


    /**
     * Function that creates XY position data set for vehicle center of mass
     *
     * @param vehiclePosition vector containing XYZ coordinates of the vehicle COM obtained during simulation
     * @param  SimTime Discrete time vector of duration of the simulation
     * @return Returns an XY series representing the relative position of the vehicle COM  in the plane for plotting
     * COM = Center of Mass
     */

    private XYDataset vehiclePositionDataSet(List<RealVector> vehiclePosition, List<Long> SimTime) {

        // Create XY Series objects for storing the XY coordinates of the vehicle
        final XYSeries vehicleXYPosition = new XYSeries("Vehicle Position", false, true);

        // Retrieving XY coordinates of the vehicle
        for(int i=0; i<vehiclePosition.size(); i++){

            vehicleXYPosition.add(vehiclePosition.get(i).getEntry(0), vehiclePosition.get(i).getEntry(1));

        }

        // Create a parallel series collection to store data
        final XYSeriesCollection vehiclePositionDataSet = new XYSeriesCollection();
        vehiclePositionDataSet.addSeries(vehicleXYPosition);

        return vehiclePositionDataSet;
    }

    /**
     * Function that creates Z position data set for vehicle center of mass
     *
     * @param wheelPositions vector containing XYZ coordinates of the vehicle COM obtained during simulation
     * @param  simTime Discrete time vector of duration of the simulation
     * @return Returns an XY series representing the relative Z position of the vehicle COM  in the plane for plotting
     * COM = Center of Mass
     */

    private XYDataset zAxisDataSet(List<List<RealVector>> wheelPositions, List<Long> simTime) {

        XYSeries wheelOneZAxis = new XYSeries("WheelOne", false, true);
        XYSeries wheelTwoZAxis = new XYSeries("WheelTwo", false, true);
        XYSeries wheelThreeZAxis = new XYSeries("WheelThree", false, true);
        XYSeries wheelFourZAxis = new XYSeries("WheelFour", false, true);
        List<Double>  temporaryOne = new LinkedList<Double>();
        List<Double>  temporaryTwo = new LinkedList<Double>();
        List<Double>  temporaryThree = new LinkedList<Double>();
        List<Double>  temporaryFour = new LinkedList<Double>();

        // Retrieving Z coordinates of the vehicle
        for(int i=0; i<wheelPositions.size(); i++){

            // storing XY coordinates in their respective series
            temporaryOne.add(wheelPositions.get(i).get(0).getEntry(2));
            temporaryTwo.add(wheelPositions.get(i).get(1).getEntry(2));
            temporaryThree.add(wheelPositions.get(i).get(2).getEntry(2));
            temporaryFour.add(wheelPositions.get(i).get(3).getEntry(2));
            wheelOneZAxis.add(simTime.get(i), temporaryOne.get(i));
            wheelTwoZAxis.add(simTime.get(i), temporaryTwo.get(i));
            wheelThreeZAxis.add(simTime.get(i), temporaryThree.get(i));
            wheelFourZAxis.add(simTime.get(i), temporaryFour.get(i));

        }

        // Create a parallel series collection to store data Z=f(t)
        final XYSeriesCollection vehicleZAxisDataSet = new XYSeriesCollection();
        vehicleZAxisDataSet.addSeries(wheelOneZAxis);
        vehicleZAxisDataSet.addSeries(wheelTwoZAxis);
        vehicleZAxisDataSet.addSeries(wheelThreeZAxis);
        vehicleZAxisDataSet.addSeries(wheelFourZAxis);

        return vehicleZAxisDataSet;
    }



    /**
     * Function that creates Velocity dataset for vehicle center of mass
     *
     * @param vehicleVelocity vector containing XYZ velocities of the vehicle COM obtained during simulation
     * @param simTime SimTime Discrete time vector of duration of the simulation
     * @return Returns an XY series representing the relative position of the vehicle COM  in the plane for plotting
     * COM = Center of Mass
     */

    private XYDataset vehicleVelocityDataSet(List<RealVector> vehicleVelocity, List<Long> simTime) {

        // Create XY Series objects for storing the velocity of the vehicle
        final XYSeries VehicleVelXY = new XYSeries("Vehicle Velocity", false, true);
        List<Double>  temporaryNorm = new LinkedList<Double>();

        // Retrieving velocity coordinates of the vehicle
        for(int i=0; i<vehicleVelocity.size(); i++){

            temporaryNorm.add(Math.sqrt(Math.pow(vehicleVelocity.get(i).getEntry(0), 2) + Math.pow(vehicleVelocity.get(i).getEntry(1), 2)));
            VehicleVelXY.add(simTime.get(i), temporaryNorm.get(i));

        }

        // Create a parallel series collection to store data
        XYSeriesCollection vehicleVelocityDataSet = new XYSeriesCollection();
            vehicleVelocityDataSet.addSeries(VehicleVelXY);

        // Return data for display
        return vehicleVelocityDataSet;
    }
}