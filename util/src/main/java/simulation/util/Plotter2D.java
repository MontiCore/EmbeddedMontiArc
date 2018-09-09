package simulation.util;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.axis.NumberAxis;

import java.awt.*;
import java.io.File;
import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.linear.RealVector;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.ui.ApplicationFrame;
import org.jfree.chart.plot.XYPlot;

import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;

/**
 * Class that Plots a 2D Graph for two data sets
 */
public class Plotter2D extends ApplicationFrame
{
    public final static int PLOTTER_OUTPUT_POSITION_XY = 1;
    public final static int PLOTTER_OUTPUT_VELOCITY = 2;
    public final static int PLOTTER_OUTPUT_Z = 3;

    /* A circle shape to be used by the plotting renderer */
    java.awt.geom.Ellipse2D.Double shape = new java.awt.geom.Ellipse2D.Double(-4.0, -4.0, 4.0, 4.0);

    /**
     * Constructor of the Plotter 2D object which responsible of creating
     * the charts for the different data sets related to the vehicle's
     * movement.
     * @param  wheelRotationRates The relative positions of the wheels respective center of mass
     * @param  vehiclePosition Relative position of the center of mass of the vehicle
     * @param  vehicleVelocity Velocity of the vehicle's center of mass
     * @param  simulationTimePoints Discrete time vector duration of the simulation
     */

    public Plotter2D(List<List<Double>> wheelRotationRates, List<RealVector> vehiclePosition, List<RealVector> vehicleVelocity, List<Long> simulationTimePoints) {
        super("Vehicle Simulation Charts");
        // Creating charts for both position and velocity based on the provided data sets
        JFreeChart positionChart = vehiclePositionChart(vehiclePosition, simulationTimePoints);
        JFreeChart velocityChart = vehicleVelocityChart(vehicleVelocity, simulationTimePoints);
        JFreeChart rotationRatesChart = wheelRotationRatesChart(wheelRotationRates, simulationTimePoints);
        try {
            ChartUtilities.saveChartAsPNG(new File("position.png"), positionChart, 750, 450);
            ChartUtilities.saveChartAsPNG(new File("velocity.png"), velocityChart, 750, 450);
            ChartUtilities.saveChartAsPNG(new File("rotationRate.png"), rotationRatesChart, 750, 450);
        }catch (Exception e){
            e.printStackTrace();
        }
    }


    /**
     * Function that creates the Vehicle position 2D plot charts
     * for both the position of the center of mass of the vehicle
     * and the four wheels centers of mass.
     *
     * @param vehiclePosition Position of the center of mass of the vehicle
     * @param  simTime Discrete time vector of duration of the simulation
     * @return Returns a JFree chart object for which a panel needs to be created for display
     */

    private JFreeChart vehiclePositionChart(List<RealVector> vehiclePosition, List<Long> simTime) {
        // Set axis names
        final NumberAxis xAxis = new NumberAxis("Simulation Time (ms)");
        final ValueAxis yAxis = new NumberAxis("Vehicle Position (m)");

        // Create position data sets for vehicle and wheel position
        final XYDataset vehiclePositionDataSet = vehiclePositionDataSet(vehiclePosition, simTime);

        // Creating and customizing renderer setting
        final XYLineAndShapeRenderer positionPlotRenderer = new XYLineAndShapeRenderer();
        positionPlotRenderer.setBaseShapesFilled(true);
        positionPlotRenderer.setSeriesPaint(0, Color.BLUE);
        positionPlotRenderer.setSeriesShape(0, shape);
        positionPlotRenderer.setSeriesLinesVisible(0, false);

        // Creating Position Plot object for wheels and adding the vehicle Center of mass
        XYPlot positionPlot = new XYPlot(vehiclePositionDataSet, xAxis, yAxis, positionPlotRenderer);
        positionPlot.getRangeAxis().setLabelFont(new Font("Dialog", Font.BOLD, 30));
        positionPlot.getRangeAxis().setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));
        positionPlot.getDomainAxis().setLabelFont(new Font("Dialog", Font.BOLD, 30));
        positionPlot.getDomainAxis().setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));

        JFreeChart positionChart = new JFreeChart(" ", JFreeChart.DEFAULT_TITLE_FONT, positionPlot, true);
        positionChart.removeLegend();

        return positionChart;
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
        final ValueAxis yAxis = new NumberAxis("Vehicle Velocity (m/s)");

        // Creating Velocity Data set
        final XYDataset velocity = vehicleVelocityDataSet(vehicleVelocity, simTime);

        // Creating and customizing renderer setting
        final XYLineAndShapeRenderer velocityPlotRenderer = new XYLineAndShapeRenderer();
        velocityPlotRenderer.setBaseShapesFilled(true);
        velocityPlotRenderer.setSeriesPaint(0, Color.BLUE);
        velocityPlotRenderer.setSeriesShape(0, shape);

        velocityPlotRenderer.setSeriesLinesVisible(0, false);

        // Creating Velocity Plot object for vehicle
        XYPlot velocityPlot = new XYPlot(velocity, xAxis, yAxis, velocityPlotRenderer);
        velocityPlot.getRangeAxis().setLabelFont(new Font("Dialog", Font.BOLD, 30));
        velocityPlot.getRangeAxis().setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));
        velocityPlot.getDomainAxis().setLabelFont(new Font("Dialog", Font.BOLD, 30));
        velocityPlot.getDomainAxis().setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));

        JFreeChart velocityChart = new JFreeChart(" ", JFreeChart.DEFAULT_TITLE_FONT, velocityPlot, true);
        velocityChart.removeLegend();

        return velocityChart;
    }

    /**
     * Function that creates the Vehicle position 2D plot charts
     * for both the position of the center of mass of the vehicle
     * and the four wheels centers of mass.
     *
     * @param wheelRotationRates Relative position of the center of mass of the vehicle
     * @param  simTime Discrete time vector of duration of the simulation
     * @return Returns a JFree chart object for which a panel needs to be created for display
     */

    private JFreeChart wheelRotationRatesChart(List<List<Double>> wheelRotationRates, List<Long> simTime) {
        // Set axis names
        final NumberAxis xAxis = new NumberAxis("Simulation Time (ms)");
        final ValueAxis yAxis = new NumberAxis("Wheel Rotation Rate (rad/s)");

        // Creating Position Data set for vehicle and wheels center of mass
        final XYDataset rotationRates = wheelRotationRatesDataSet(wheelRotationRates, simTime);

        // Creating and customizing renderer setting
        final XYLineAndShapeRenderer rotationRatesPlotRenderer = new XYLineAndShapeRenderer();

        rotationRatesPlotRenderer.setBaseShapesFilled(true);
        rotationRatesPlotRenderer.setSeriesPaint(0, Color.BLUE);
        rotationRatesPlotRenderer.setSeriesShape(0, shape);
        rotationRatesPlotRenderer.setSeriesPaint(1, Color. RED);
        rotationRatesPlotRenderer.setSeriesShape(1, shape);
        rotationRatesPlotRenderer.setSeriesPaint(2, Color.GREEN);
        rotationRatesPlotRenderer.setSeriesShape(2, shape);
        rotationRatesPlotRenderer.setSeriesPaint(3, Color.YELLOW);
        rotationRatesPlotRenderer.setSeriesShape(3, shape);

        rotationRatesPlotRenderer.setSeriesLinesVisible(0, false);
        rotationRatesPlotRenderer.setSeriesLinesVisible(1, false);
        rotationRatesPlotRenderer.setSeriesLinesVisible(2, false);
        rotationRatesPlotRenderer.setSeriesLinesVisible(3, false);

        // Creating Position Plot object for wheels and adding the vehicle Center of mass
        XYPlot rotationRatePlot  = new XYPlot(rotationRates, xAxis, yAxis, rotationRatesPlotRenderer);
        rotationRatePlot.getRangeAxis().setLabelFont(new Font("Dialog", Font.BOLD, 30));
        rotationRatePlot.getRangeAxis().setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));
        rotationRatePlot.getDomainAxis().setLabelFont(new Font("Dialog", Font.BOLD, 30));
        rotationRatePlot.getDomainAxis().setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));

        JFreeChart rotationRateChart = new JFreeChart(" ", JFreeChart.DEFAULT_TITLE_FONT, rotationRatePlot, true);
        rotationRateChart.removeLegend();

        return rotationRateChart;
    }


    /**
     * Function that creates XY position data set for vehicle center of mass
     *
     * @param vehiclePosition vector containing XYZ coordinates of the vehicle COM obtained during simulation
     * @param  simTime Discrete time vector of duration of the simulation
     * @return Returns an XY series representing the relative position of the vehicle COM  in the plane for plotting
     * COM = Center of Mass
     */
    private XYDataset vehiclePositionDataSet(List<RealVector> vehiclePosition, List<Long> simTime) {

        // Create XY Series objects for storing the XY coordinates of the vehicle
        final XYSeries vehicleYPosition = new XYSeries("p_y", false, true);
        List<Double> tempPosition = new LinkedList<Double>();

        // Retrieving XY coordinates of the vehicle
        for(int i=0; i<vehiclePosition.size(); i++){

            tempPosition.add(vehiclePosition.get(i).getEntry(1));
            vehicleYPosition.add(simTime.get(i), tempPosition.get(i));

        }

        // Create a parallel series collection to store data
        final XYSeriesCollection vehiclePositionDataSet = new XYSeriesCollection();
        vehiclePositionDataSet.addSeries(vehicleYPosition);

        return vehiclePositionDataSet;
    }


    /**
     * Function that creates Velocity dataset for vehicle center of mass
     *
     * @param vehicleVelocity vector containing XYZ velocities of the vehicle COM obtained during simulation
     * @param simTime Discrete time vector of duration of the simulation
     * @return Returns an XY series representing the relative position of the vehicle COM  in the plane for plotting
     * COM = Center of Mass
     */

    private XYDataset vehicleVelocityDataSet(List<RealVector> vehicleVelocity, List<Long> simTime) {

        // Create XY Series objects for storing the velocity of the vehicle
        final XYSeries VehicleVelXY = new XYSeries("v_y", false, true);
        List<Double>  temporaryNorm = new LinkedList<Double>();

        // Retrieving velocity coordinates of the vehicle
        for(int i=0; i<vehicleVelocity.size(); i++){

            temporaryNorm.add(vehicleVelocity.get(i).getEntry(1));
            VehicleVelXY.add(simTime.get(i), temporaryNorm.get(i));

        }

        // Create a parallel series collection to store data
        XYSeriesCollection vehicleVelocityDataSet = new XYSeriesCollection();
            vehicleVelocityDataSet.addSeries(VehicleVelXY);

        // Return data for display
        return vehicleVelocityDataSet;
    }


    /**
     * Function that creates Z position data set for vehicle center of mass
     *
     * @param wheelRotationRates vector containing XYZ coordinates of the vehicle COM obtained during simulation
     * @param  simTime Discrete time vector of duration of the simulation
     * @return Returns an XY series representing the relative Z position of the vehicle COM  in the plane for plotting
     * COM = Center of Mass
     */

    private XYDataset wheelRotationRatesDataSet(List<List<Double>> wheelRotationRates, List<Long> simTime) {

        XYSeries wheelOne = new XYSeries("omega_wheel_1", false, true);
        XYSeries wheelTwo = new XYSeries("omega_wheel_2", false, true);
        XYSeries wheelThree = new XYSeries("omega_wheel_3", false, true);
        XYSeries wheelFour = new XYSeries("omega_wheel_4", false, true);
        List<Double> temporaryOne = new LinkedList<Double>();
        List<Double> temporaryTwo = new LinkedList<Double>();
        List<Double> temporaryThree = new LinkedList<Double>();
        List<Double> temporaryFour = new LinkedList<Double>();

        // Retrieving Z coordinates of the vehicle
        for(int i=0; i<wheelRotationRates.size(); i++){

            temporaryOne.add(wheelRotationRates.get(i).get(0));
            temporaryTwo.add(wheelRotationRates.get(i).get(1));
            temporaryThree.add(wheelRotationRates.get(i).get(2));
            temporaryFour.add(wheelRotationRates.get(i).get(3));
            wheelOne.add(simTime.get(i), temporaryOne.get(i));
            wheelTwo.add(simTime.get(i), temporaryTwo.get(i));
            wheelThree.add(simTime.get(i), temporaryThree.get(i));
            wheelFour.add(simTime.get(i), temporaryFour.get(i));

        }

        // Create a parallel series collection to store data
        final XYSeriesCollection vehicleZAxisDataSet = new XYSeriesCollection();
        vehicleZAxisDataSet.addSeries(wheelOne);
        vehicleZAxisDataSet.addSeries(wheelTwo);
        vehicleZAxisDataSet.addSeries(wheelThree);
        vehicleZAxisDataSet.addSeries(wheelFour);

        return vehicleZAxisDataSet;
    }
}