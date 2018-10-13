package simulation.util;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.axis.NumberAxis;

import java.awt.*;
import java.io.File;
import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.linear.RealVector;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberTickUnit;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.chart.plot.XYPlot;

import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;

/**
 * Class that Plots a 2D Graph for data sets
 */
public final class Plotter2D{

    /* A circle shape to be used by the plotting renderer */
    private static java.awt.geom.Ellipse2D.Double shape = new java.awt.geom.Ellipse2D.Double(-2.0, -2.0, 4.0, 4.0);

    private static void plotOne(List<RealVector> vehiclePosition, List<Long> simulationTime, int i, List<List<RealVector>> wheelPositions){
        if(i % 10 != 0){
            //return;
        }
        // Create axis
        NumberAxis xAxis = new NumberAxis("Vehicle Position (m)");
        xAxis.setLabelFont(new Font("Dialog", Font.BOLD, 30));
        xAxis.setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));
        xAxis.setTickLabelsVisible(false);
        xAxis.setTickUnit(new NumberTickUnit(2));
        xAxis.setRange(vehiclePosition.get(i).getEntry(0) - 5.0, vehiclePosition.get(i).getEntry(0) + 5.0);

        NumberAxis yAxis = new NumberAxis("Vehicle Position (m)");
        yAxis.setLabelFont(new Font("Dialog", Font.BOLD, 30));
        yAxis.setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));
        yAxis.setTickLabelsVisible(false);
        yAxis.setTickUnit(new NumberTickUnit(2));
        yAxis.setRange(vehiclePosition.get(i).getEntry(1) - 5.0, vehiclePosition.get(i).getEntry(1) + 5.0);

        // Create XY series for storing the position
        XYSeries position = new XYSeries("x_y", false, true);
        double tempPosition = 0.0;

        tempPosition = vehiclePosition.get(i).getEntry(1);
        position.add(vehiclePosition.get(i).getEntry(0), tempPosition);

        tempPosition = vehiclePosition.get(i + 1).getEntry(1);
        position.add(vehiclePosition.get(i + 1).getEntry(0), tempPosition);

        // Create XY series for storing the wheel positions
        XYSeries position1 = new XYSeries("x_y_1", false, true);
        XYSeries position2 = new XYSeries("x_y_2", false, true);
        XYSeries position3 = new XYSeries("x_y_3", false, true);
        XYSeries position4 = new XYSeries("x_y_4", false, true);

        tempPosition = wheelPositions.get(i).get(0).getEntry(1);
        position1.add(wheelPositions.get(i).get(0).getEntry(0), tempPosition);
        tempPosition = wheelPositions.get(i).get(1).getEntry(1);
        position2.add(wheelPositions.get(i).get(1).getEntry(0), tempPosition);
        tempPosition = wheelPositions.get(i).get(2).getEntry(1);
        position3.add(wheelPositions.get(i).get(2).getEntry(0), tempPosition);
        tempPosition = wheelPositions.get(i).get(3).getEntry(1);
        position4.add(wheelPositions.get(i).get(3).getEntry(0), tempPosition);

        // Create a parallel series collection and store data
        XYSeriesCollection vehiclePositionDataSet = new XYSeriesCollection();
        vehiclePositionDataSet.addSeries(position);
        vehiclePositionDataSet.addSeries(position1);
        vehiclePositionDataSet.addSeries(position2);
        vehiclePositionDataSet.addSeries(position3);
        vehiclePositionDataSet.addSeries(position4);

        // Create and customize renderer
        XYLineAndShapeRenderer positionPlotRenderer = new XYLineAndShapeRenderer();
        positionPlotRenderer.setBaseShapesFilled(true);

        positionPlotRenderer.setSeriesPaint(0, Color.BLUE);
        positionPlotRenderer.setSeriesShape(0, shape);
        positionPlotRenderer.setSeriesLinesVisible(0, true);

        positionPlotRenderer.setSeriesPaint(1, Color.GREEN);
        positionPlotRenderer.setSeriesShape(1, shape);
        positionPlotRenderer.setSeriesLinesVisible(1, false);

        positionPlotRenderer.setSeriesPaint(2, Color.GREEN);
        positionPlotRenderer.setSeriesShape(2, shape);
        positionPlotRenderer.setSeriesLinesVisible(2, false);

        positionPlotRenderer.setSeriesPaint(3, Color.RED);
        positionPlotRenderer.setSeriesShape(3, shape);
        positionPlotRenderer.setSeriesLinesVisible(3, false);

        positionPlotRenderer.setSeriesPaint(4, Color.RED);
        positionPlotRenderer.setSeriesShape(4, shape);
        positionPlotRenderer.setSeriesLinesVisible(4, false);

        // Creating position plot
        XYPlot positionPlot = new XYPlot(vehiclePositionDataSet, xAxis, yAxis, positionPlotRenderer);
        positionPlot.setDomainGridlinePaint(Color.BLACK);
        positionPlot.setRangeGridlinePaint(Color.BLACK);

        // Create position chart
        JFreeChart positionChart = new JFreeChart(" ", JFreeChart.DEFAULT_TITLE_FONT, positionPlot, true);
        positionChart.removeLegend();

        try {
            ChartUtilities.saveChartAsPNG(new File("Position" + i + ".png"), positionChart, 750, 750);
        }catch (Exception e){
            Log.severe("Could not save charts as PNGs." + e);
        }
    }

    public static void plotMany(List<RealVector> vehiclePosition, List<Long> simulationTimePoints, List<List<RealVector>> wheelPositions){

        for(int i=0; i<vehiclePosition.size() - 1; i++){

            plotOne(vehiclePosition, simulationTimePoints, i, wheelPositions);

        }
    }

    /**
     * Creates charts from the given data sets and saves them as images
     *
     * @param  vehiclePosition Position of the center of mass of the vehicle
     * @param  vehicleVelocity Velocity of the center of mass of the vehicle
     * @param  wheelRotationRates Rotation rates of the wheels of the vehicle
     * @param  simulationTimePoints Discrete time points during the simulation
     */

    public static void plot(List<RealVector> vehiclePosition, List<RealVector> vehicleVelocity, List<List<Double>> wheelRotationRates, List<Long> simulationTimePoints) {
        JFreeChart positionChart = vehiclePositionChart(vehiclePosition, simulationTimePoints);
        JFreeChart velocityChart = vehicleVelocityChart(vehicleVelocity, simulationTimePoints);
        JFreeChart rotationRatesChart = wheelRotationRatesChart(wheelRotationRates, simulationTimePoints);
        try {
            ChartUtilities.saveChartAsPNG(new File("Position.png"), positionChart, 750, 450);
            ChartUtilities.saveChartAsPNG(new File("Velocity.png"), velocityChart, 750, 450);
            ChartUtilities.saveChartAsPNG(new File("Rotation.png"), rotationRatesChart, 750, 450);
        }catch (Exception e){
            Log.severe("Could not save charts as PNGs." + e);
        }
    }


    /**
     * Function that creates the vehicle position chart
     *
     * @param  vehiclePosition Position of the center of mass of the vehicle
     * @param  simTime Discrete time points during the simulation
     * @return Returns a chart of the position of the center of mass of the vehicle
     */

    private static JFreeChart vehiclePositionChart(List<RealVector> vehiclePosition, List<Long> simTime) {
        // Create axis
        NumberAxis xAxis = new NumberAxis("Simulation Time (ms)");
        xAxis.setLabelFont(new Font("Dialog", Font.BOLD, 30));
        xAxis.setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));

        ValueAxis yAxis = new NumberAxis("Vehicle Position (m)");
        yAxis.setLabelFont(new Font("Dialog", Font.BOLD, 30));
        yAxis.setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));

        // Create position data set
        XYDataset vehiclePositionDataSet = vehiclePositionDataSet(vehiclePosition, simTime);

        // Create and customize renderer
        XYLineAndShapeRenderer positionPlotRenderer = new XYLineAndShapeRenderer();
        positionPlotRenderer.setBaseShapesFilled(true);
        positionPlotRenderer.setSeriesPaint(0, Color.BLUE);
        positionPlotRenderer.setSeriesShape(0, shape);
        positionPlotRenderer.setSeriesLinesVisible(0, false);

        // Creating position plot
        XYPlot positionPlot = new XYPlot(vehiclePositionDataSet, xAxis, yAxis, positionPlotRenderer);

        // Create position chart
        JFreeChart positionChart = new JFreeChart(" ", JFreeChart.DEFAULT_TITLE_FONT, positionPlot, true);
        positionChart.removeLegend();

        return positionChart;
    }

    /**
     * Function that creates the vehicle velocity chart
     *
     * @param  vehicleVelocity Velocity of the center of mass of the vehicle
     * @param  simTime Discrete time points during the simulation
     * @return Returns a chart of the velocity of the center of mass of the vehicle
     */
    private static JFreeChart vehicleVelocityChart(List<RealVector> vehicleVelocity, List<Long> simTime ) {
        // Create axis
        NumberAxis xAxis = new NumberAxis("Simulation Time (ms)");
        xAxis.setLabelFont(new Font("Dialog", Font.BOLD, 30));
        xAxis.setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));

        ValueAxis yAxis = new NumberAxis("Vehicle Velocity (m/s)");
        yAxis.setLabelFont(new Font("Dialog", Font.BOLD, 30));
        yAxis.setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));
        yAxis.setRange(0.0, 17.5);

        // Create velocity data set
        XYDataset velocity = vehicleVelocityDataSet(vehicleVelocity, simTime);

        // Create and customize renderer
        XYLineAndShapeRenderer velocityPlotRenderer = new XYLineAndShapeRenderer();
        velocityPlotRenderer.setBaseShapesFilled(true);
        velocityPlotRenderer.setSeriesPaint(0, Color.BLUE);
        velocityPlotRenderer.setSeriesShape(0, shape);
        velocityPlotRenderer.setSeriesLinesVisible(0, false);

        // Creating velocity plot
        XYPlot velocityPlot = new XYPlot(velocity, xAxis, yAxis, velocityPlotRenderer);

        // Create position chart
        JFreeChart velocityChart = new JFreeChart(" ", JFreeChart.DEFAULT_TITLE_FONT, velocityPlot, true);
        velocityChart.removeLegend();

        return velocityChart;
    }

    /**
     * Function that creates the vehicle rotation rates chart
     *
     * @param  wheelRotationRates Rotation rates of the wheels of the vehicle
     * @param  simTime Discrete time points during the simulation
     * @return Returns a chart of the rotation rates of the wheels of the vehicle
     */

    private static JFreeChart wheelRotationRatesChart(List<List<Double>> wheelRotationRates, List<Long> simTime) {
        // Create axis
        NumberAxis xAxis = new NumberAxis("Simulation Time (ms)");
        xAxis.setLabelFont(new Font("Dialog", Font.BOLD, 30));
        xAxis.setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));

        ValueAxis yAxis = new NumberAxis("Wheel Rotation Rate (rad/s)");
        yAxis.setLabelFont(new Font("Dialog", Font.BOLD, 30));
        yAxis.setTickLabelFont(new Font("Dialog", Font.PLAIN, 25));

        // Create rotation rates data set
        XYDataset rotationRates = wheelRotationRatesDataSet(wheelRotationRates, simTime);

        // Create and customize renderer
        XYLineAndShapeRenderer rotationRatesPlotRenderer = new XYLineAndShapeRenderer();

        rotationRatesPlotRenderer.setBaseShapesFilled(true);

        rotationRatesPlotRenderer.setSeriesPaint(0, Color.BLUE);
        rotationRatesPlotRenderer.setSeriesPaint(1, Color. RED);
        rotationRatesPlotRenderer.setSeriesPaint(2, Color.GREEN);
        rotationRatesPlotRenderer.setSeriesPaint(3, Color.YELLOW);

        rotationRatesPlotRenderer.setSeriesShape(0, shape);
        rotationRatesPlotRenderer.setSeriesShape(1, shape);
        rotationRatesPlotRenderer.setSeriesShape(2, shape);
        rotationRatesPlotRenderer.setSeriesShape(3, shape);

        rotationRatesPlotRenderer.setSeriesLinesVisible(0, false);
        rotationRatesPlotRenderer.setSeriesLinesVisible(1, false);
        rotationRatesPlotRenderer.setSeriesLinesVisible(2, false);
        rotationRatesPlotRenderer.setSeriesLinesVisible(3, false);

        // Create rotation rates plot
        XYPlot rotationRatePlot  = new XYPlot(rotationRates, xAxis, yAxis, rotationRatesPlotRenderer);

        // Create rotation rates chart
        JFreeChart rotationRateChart = new JFreeChart(" ", JFreeChart.DEFAULT_TITLE_FONT, rotationRatePlot, true);
        rotationRateChart.removeLegend();

        return rotationRateChart;
    }


    /**
     * Function that creates the vehicle position XY data set
     *
     * @param  vehiclePosition Position of the center of mass of the vehicle+
     * @param  simTime Discrete time points during the simulation
     * @return Returns a XY date set of the position of the center of mass of the vehicle
     */
    private static XYDataset vehiclePositionDataSet(List<RealVector> vehiclePosition, List<Long> simTime) {

        // Create XY series for storing the position
        XYSeries position = new XYSeries("x_y", false, true);
        List<Double> tempPosition = new LinkedList<Double>();

        // Fill series with data
        for(int i=0; i<vehiclePosition.size(); i++){

            tempPosition.add(vehiclePosition.get(i).getEntry(1));
            //position.add(simTime.get(i), tempPosition.get(i));
            position.add(vehiclePosition.get(i).getEntry(0), tempPosition.get(i));

        }

        // Create a parallel series collection and store data
        XYSeriesCollection vehiclePositionDataSet = new XYSeriesCollection();
        vehiclePositionDataSet.addSeries(position);

        return vehiclePositionDataSet;
    }


    /**
     * Function that creates the vehicle velocity XY data set
     *
     * @param  vehicleVelocity Velocity of the center of mass of the vehicle
     * @param  simTime Discrete time points during the simulation
     * @return Returns a XY date set of the velocity of the center of mass of the vehicle
     */

    private static XYDataset vehicleVelocityDataSet(List<RealVector> vehicleVelocity, List<Long> simTime) {

        // Create XY series for storing the velocity
        XYSeries velocity = new XYSeries("v_y", false, true);
        List<Double>  tempVelocity = new LinkedList<Double>();

        // Fill series with data
        for(int i=0; i<vehicleVelocity.size(); i++){

            tempVelocity.add(vehicleVelocity.get(i).getEntry(1));
            velocity.add(simTime.get(i), tempVelocity.get(i));

        }

        // Create a parallel series collection and store data
        XYSeriesCollection vehicleVelocityDataSet = new XYSeriesCollection();
        vehicleVelocityDataSet.addSeries(velocity);

        return vehicleVelocityDataSet;
    }


    /**
     * Function that creates the vehicle rotation rates XY data set
     *
     * @param  wheelRotationRates Rotation rates of the wheels of the vehicle
     * @param  simTime Discrete time points during the simulation
     * @return Returns a XY date set of the rotation rates of the wheels of the vehicle
     */

    private static XYDataset wheelRotationRatesDataSet(List<List<Double>> wheelRotationRates, List<Long> simTime) {

        // Create XY series for storing the wheel rotation rates
        XYSeries wheelOne = new XYSeries("omega_wheel_1", false, true);
        XYSeries wheelTwo = new XYSeries("omega_wheel_2", false, true);
        XYSeries wheelThree = new XYSeries("omega_wheel_3", false, true);
        XYSeries wheelFour = new XYSeries("omega_wheel_4", false, true);
        List<Double> tempOne = new LinkedList<Double>();
        List<Double> tempTwo = new LinkedList<Double>();
        List<Double> tempThree = new LinkedList<Double>();
        List<Double> tempFour = new LinkedList<Double>();

        // Fill series with data
        for(int i=0; i<wheelRotationRates.size(); i++){

            tempOne.add(wheelRotationRates.get(i).get(0));
            tempTwo.add(wheelRotationRates.get(i).get(1));
            tempThree.add(wheelRotationRates.get(i).get(2));
            tempFour.add(wheelRotationRates.get(i).get(3));
            wheelOne.add(simTime.get(i), tempOne.get(i));
            wheelTwo.add(simTime.get(i), tempTwo.get(i));
            wheelThree.add(simTime.get(i), tempThree.get(i));
            wheelFour.add(simTime.get(i), tempFour.get(i));

        }

        // Create a parallel series collection and store data
        XYSeriesCollection vehicleRotationRatesDataSet = new XYSeriesCollection();
        vehicleRotationRatesDataSet.addSeries(wheelOne);
        vehicleRotationRatesDataSet.addSeries(wheelTwo);
        vehicleRotationRatesDataSet.addSeries(wheelThree);
        vehicleRotationRatesDataSet.addSeries(wheelFour);

        return vehicleRotationRatesDataSet;
    }
}