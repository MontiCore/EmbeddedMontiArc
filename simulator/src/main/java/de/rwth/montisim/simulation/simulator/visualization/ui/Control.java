/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.simulator.visualization.ui;

import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.Timer;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.Time;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.time.Duration;
import java.time.Instant;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.Locale;

public class Control extends JPanel implements ActionListener, ChangeListener {
    private static final long serialVersionUID = 2776373934381177954L;

    public static final ImageIcon playIcon;
    public static final ImageIcon pauseIcon;
    public static final ImageIcon stepForwardIcon;
    public static final ImageIcon stepBackIcon;
    public static final ImageIcon resetIcon;
    static {
        playIcon = createImageIcon("/images/play.png");
        pauseIcon = createImageIcon("/images/pause.png");
        stepForwardIcon = createImageIcon("/images/step-forward.png");
        stepBackIcon = createImageIcon("/images/step-back.png");
        resetIcon = createImageIcon("/images/reset.png");
    }

    DateTimeFormatter timeFormatter =
    DateTimeFormatter.ofPattern("HH:mm:ss.SSS")
                     .withLocale( Locale.UK )
                     .withZone( ZoneId.systemDefault() );

    private static final DecimalFormat ratioFormatter = new DecimalFormat("##0.000",
        DecimalFormatSymbols.getInstance(Locale.ENGLISH));

    public static enum Mode {
        SIMULATION, REPLAY
    }

    private static enum SimulationMode {
        REAL_TIME("Real Time"), FULL_SPEED("Full Speed");

        String name;

        SimulationMode(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    private static enum SimulationSpeed {
        TIMES16("x 16", 16), TIMES8("x 8", 8), TIMES4("x 4", 4), TIMES2("x 2", 2), REAL_TIME("x 1", 1),
        HALF("x 1/2", 0.5), FOURTH("x 1/4", 0.25), EIGHTH("x 1/8", 0.125), SIXTEENTH("x 1/16", 0.0625),
        DIV50("x 1/50", 0.02), DIV100("x 1/100", 0.01);

        String name;
        public final double factor;

        SimulationSpeed(String name, double factor) {
            this.name = name;
            this.factor = factor;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    private final static SimulationMode simModes[] = { SimulationMode.REAL_TIME, SimulationMode.FULL_SPEED };
    private final static SimulationSpeed simSpeeds[] = { SimulationSpeed.TIMES16, SimulationSpeed.TIMES8,
            SimulationSpeed.TIMES4, SimulationSpeed.TIMES2, SimulationSpeed.REAL_TIME, SimulationSpeed.HALF,
            SimulationSpeed.FOURTH, SimulationSpeed.EIGHTH, SimulationSpeed.SIXTEENTH, SimulationSpeed.DIV50,
            SimulationSpeed.DIV100 };

    /*
     * SWING COMPONENTS
     */

    final JButton resetButton;
    final JComboBox<SimulationMode> simModeButton;
    final JComboBox<SimulationSpeed> speedButton;

    final JButton stepBackButton;
    final JButton playPauseButton;
    final JButton stepForwardButton;

    final JLabel timeLabel;
    //https://docs.oracle.com/javase/tutorial/uiswing/components/slider.html
    final JSlider slider;
    final JLabel deltaTLabel;
    final JLabel simSpeedLabel;

    final JLabel fpsLabel;
    // final JLabel frameDurationLabel;
    // final JLabel frameVarianceLabel;
    
    // final JLabel manualFPSLabel;

    /*
     * Simulation/Replay properties
     */

    final private Mode mode;

    double speed = 1;
    boolean fullspeed = false;

    Instant simulationStart;
    long simulationTimeNano = 0;
    long simulationRealTime = 0;
    long deltaT = 0;

    final SimulationRunner runner;

    final FrameCounter counter;
    int manualCounter = 0;
    long lastFPSTime = 0;

    /*
     * Timer system
     */
    
    // TODO set physics update time as parameter
    final long PHYSICS_TICK_DURATION_MS = 10;
    final long TARGET_FPS = 30;

    final double TICK_DURATION = PHYSICS_TICK_DURATION_MS / 1000.0;
    final double FRAME_DURATION = 1.0 / TARGET_FPS;
    final long FRAME_DURATION_NANO = (Time.SECOND_TO_NANOSEC) / TARGET_FPS;
    final long TICK_NANO = PHYSICS_TICK_DURATION_MS * 1000000;
    TimeUpdate tu = new TimeUpdate(Instant.EPOCH, Duration.ofMillis(PHYSICS_TICK_DURATION_MS));

    final Timer timer;
    private boolean playing = false;
    long lastFrameTime = 0;
    long lastUpdate = 0;
    long lastOvershoot = 0;

    /*
     * Setup the Control Component
     */

    public Control(Mode mode, Instant simulationStart, SimulationRunner runner) {
        this.mode = mode;
        this.simulationStart = simulationStart;
        this.runner = runner;
        this.counter = new FrameCounter(30);

        timer = new Timer((int)(FRAME_DURATION*1000), this);

        setLayout(new FlowLayout());
        // setBackground(new Color(255,255,255));

        
        simModeButton = new JComboBox<>(simModes);
        simModeButton.addActionListener(this);
        if (mode == Mode.SIMULATION)
            add(simModeButton);

        speedButton = new JComboBox<>(simSpeeds);
        speedButton.setSelectedIndex(4);
        speedButton.setToolTipText("Simulation Speed");
        speedButton.addActionListener(this);
        add(speedButton);

        resetButton = new JButton("", resetIcon);
        resetButton.addActionListener(this);
        resetButton.setToolTipText("Reset");
        add(resetButton);

        stepBackButton = new JButton("", stepBackIcon);
        stepBackButton.addActionListener(this);
        stepBackButton.setToolTipText("Step Back");
        if (mode == Mode.REPLAY)
            add(stepBackButton);

        playPauseButton = new JButton("", playIcon);
        if (mode == Mode.REPLAY)
            playPauseButton.setToolTipText("Play/Pause the Replay");
        else
            playPauseButton.setToolTipText("Play/Pause the Simulation");
        playPauseButton.addActionListener(this);
        add(playPauseButton);

        stepForwardButton = new JButton("", stepForwardIcon);
        stepForwardButton.setToolTipText("Step Forward");
        stepForwardButton.addActionListener(this);
        add(stepForwardButton);

        // TODO labels
        add(new JLabel("Time:"));
        timeLabel = new JLabel(printTime(simulationStart));
        add(timeLabel);

        slider = new JSlider(JSlider.HORIZONTAL);
        slider.addChangeListener(this);
        if (mode == Mode.REPLAY)
            add(slider);

        add(new JLabel("\u0394t="));
        deltaTLabel = new JLabel("-");
        add(deltaTLabel);

        add(new JLabel("Sim Speed="));
        simSpeedLabel = new JLabel("-");
        add(simSpeedLabel);

        add(new JLabel("FPS="));
        fpsLabel = new JLabel("-");
        add(fpsLabel);
        
        // JLabel frameDurationText = new JLabel("FD=");
        // frameDurationText.setToolTipText("Average Frame Duration (Last "+counter.trackedFrames+" frames)");
        // add(frameDurationText);
        // frameDurationLabel = new JLabel("-");
        // add(frameDurationLabel);
        
        // JLabel frameVarText = new JLabel("V(FD)=");
        // frameVarText.setToolTipText("Frame Duration Variance");
        // add(frameVarText);
        // frameVarianceLabel = new JLabel("-");
        // add(frameVarianceLabel);
        
        // add(new JLabel("FPS'="));
        // manualFPSLabel = new JLabel("-");
        // add(manualFPSLabel);
    }

    /*
     * Handle the Component Actions
     */

    @Override
    public void actionPerformed(ActionEvent e) {
        Object s = e.getSource();
        if (s == timer) {
            if (mode == Mode.SIMULATION){
                updateSimulation();
            }
            else {
                // TODO
            }
        }
        else if (s == simModeButton) {
            //System.out.println("Sim Mode: " + ((SimulationMode) simModeButton.getSelectedItem()).name);
            if (((SimulationMode) simModeButton.getSelectedItem()) == SimulationMode.FULL_SPEED){
                if (!fullspeed){
                    fullspeed = true;
                    this.remove(1);
                    this.revalidate();
                }
            } else {
                if (fullspeed){
                    fullspeed = false;
                    this.add(speedButton, 1);
                    this.revalidate();
                }
            }
        } else if (s == speedButton) {
            //System.out.println("Speed: " + ((SimulationSpeed) speedButton.getSelectedItem()).name);
            speed = ((SimulationSpeed) speedButton.getSelectedItem()).factor;
        } else if (s == resetButton) {
            reset();
        } else if (s == stepBackButton) {
            if (playing) return;
            System.out.println("Step Back");
            // TODO
        } else if (s == playPauseButton) {
            if (playing) pause();
            else play();
        } else if (s == stepForwardButton) {
            if (playing) return;
            System.out.println("Step Forward");
            // TODO
        }
    }

    @Override
    public void stateChanged(ChangeEvent e) {
        if (e.getSource() == slider){
            //System.out.println("Slider: "+slider.getValue());
            if (!slider.getValueIsAdjusting()){
                // TODO
            }
        }
    }

    /*
     * Simulation Control
     */

    private void updateSimulation(){
            if ((System.nanoTime() - lastUpdate) < 1000000) return; // Avoid timer calls stacking up if slow simulation
            // Simulate
            boolean stopSim = false;
            long newDeltaT = 0;
            long time;
            long target;

            long simStartTime = simulationTimeNano;
            long frameStartTime = lastFrameTime;

            if (fullspeed){
                time = System.nanoTime();
                target = time + FRAME_DURATION_NANO;
            } else {
                lastFrameTime = System.nanoTime();
                target = lastFrameTime - frameStartTime;
                time = lastOvershoot; // Time in "Observed Time"
            }

            while(time < target) {
                long dt = runner.run(simulationStart.plus(Duration.ofNanos(simulationTimeNano)));
                if (dt == 0){
                    stopSim = true;
                    break;
                }
                newDeltaT = dt;
                simulationTimeNano += dt;


                if (fullspeed){
                    time = System.nanoTime();
                } else {
                    time += (long) (dt/speed);
                    lastFrameTime = System.nanoTime();
                    target = lastFrameTime - frameStartTime;
                }
            }
            lastOvershoot = time-target;

            long computationTime = lastFrameTime - frameStartTime;
            long simulatedTime = simulationTimeNano - simStartTime;

            counter.addFrame(lastFrameTime);

            updateFPS();

            
            // manualCounter++;
            // if (lastFrameTime-lastFPSTime > Time.SECOND_TO_NANOSEC){
            //     long fps = manualCounter*Time.SECOND_TO_NANOSEC/(lastFrameTime-lastFPSTime);
            //     manualFPSLabel.setText(Long.toString(fps));
            //     lastFPSTime = lastFrameTime;
            //     manualCounter = 0;
            // }
        

            updateTimeLabel();
            updateDeltaTLabel(newDeltaT);
            updateSimSpeedLabel(simulatedTime / (double) computationTime);

            runner.redraw();

            lastUpdate = System.nanoTime();

            if (stopSim) pause();
    }

    public void play() {
        if (playing) return;
        playing = true;
        playPauseButton.setIcon(pauseIcon);
        //System.out.println("Play");
        timer.start();
        lastUpdate = System.nanoTime();
        lastFrameTime = lastUpdate;
        lastFPSTime = lastUpdate;
        lastOvershoot = 0;
    }

    public void pause() {
        if (!playing) return;
        playing = false;
        playPauseButton.setIcon(playIcon);
        //System.out.println("Pause");
        timer.stop();
    }

    private void reset(){
        runner.reset();
        this.simulationTimeNano = 0;
        updateTimeLabel();
        runner.redraw();
    }


    private String printTime(Instant t){
        return timeFormatter.format(t);
    }

    private void updateTimeLabel(){
        timeLabel.setText(printTime(simulationStart.plus(Duration.ofNanos(simulationTimeNano))));
    }

    private void updateDeltaTLabel(long newDeltaT){
        if (newDeltaT != deltaT){
            deltaT = newDeltaT;
            deltaTLabel.setText(Long.toString(deltaT/1000000)+"ms");
        }
    }

    private void updateSimSpeedLabel(double speedRatio){
        simSpeedLabel.setText(ratioFormatter.format(speedRatio));
    }

    private void updateFPS(){
        long avg = counter.getAvgDelta();
        if (avg != 0){
            double fps = counter.getFramesPerSeconds();
            fpsLabel.setText(ratioFormatter.format(fps));
            // frameDurationLabel.setText(Long.toString(avg/1000000)+"ms");
            // long var = Math.round(Math.sqrt(counter.getVariance()));
            // frameVarianceLabel.setText(Long.toString(var/1000)+"us");
        }
    }

    
    protected static ImageIcon createImageIcon(String path) {
        java.net.URL imgURL = Control.class.getResource(path);
        if (imgURL != null)
            return new ImageIcon(imgURL);

        // System.out.println("wd: " + LibraryService.getWorkingDirectory());
        // TODO error handling ?
        return null;
    }
}

class FrameCounter {
    final int trackedFrames;
    final long deltas[];
    long lastTime;
    int currentPos = 0;
    int count = 0;
    long sum = 0;
    long sumSquared = 0;

    public FrameCounter(int trackedFrames){
        this.trackedFrames = trackedFrames;
        this.deltas = new long[trackedFrames];
    }

    public void addFrame(long timestamp){
        if (count == 0){
            lastTime = timestamp;
            count = 1;
            return;
        }
        long delta = timestamp-lastTime;
        if (count == trackedFrames){
            long last = deltas[currentPos];
            sum += delta - last;
            sumSquared += (delta*delta) - (last*last);
        } else {
            ++count;
            sum += delta;
            sumSquared += (delta*delta);
        }
        deltas[currentPos] = delta;
        lastTime = timestamp;
        currentPos = (currentPos+1)%trackedFrames;
    }

    public long getAvgDelta(){
        if (count == 0) return 0;
        else return sum/count;
    }

    public long getVariance(){
        if (count == 0) return 0;
        long avg = getAvgDelta();
        return sumSquared/count - avg*avg;
    }

    public double getFramesPerSeconds(){
        long a = getAvgDelta();
        if (a == 0) return 0;
        return Time.SECOND_TO_NANOSEC/(double) a;
    }
}