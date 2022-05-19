/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.ui;

import javax.swing.*;
import javax.swing.event.*;
import java.awt.FlowLayout;
import java.awt.Color;
import java.awt.event.*;
import java.text.*;
import java.time.*;
import java.time.format.DateTimeFormatter;
import java.util.Locale;

import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.Time;

public class Control extends JPanel implements ActionListener, ChangeListener {
    private static final long serialVersionUID = 2776373934381177954L;
    public static final Color RUNNING_COLOR = new Color(47, 141, 235);
    public static final Color SUCCEEDED_COLOR = Color.GREEN;
    public static final Color FAILED_COLOR = Color.RED;

    /*
     * Simulation/Replay properties
     */

    final private Mode mode;

    double simulationSpeed = 1;
    boolean fullspeed = false;

    // Instant simulationStart;
    // long simulationTimeNano = 0;
    // long simulationRealTime = 0;
    // long deltaT = 0;

    final SimulationRunner runner;

    final FrameCounter counter;
    int manualCounter = 0;
    long lastFPSTime = 0;

    /*
     * Timer system
     */

    Instant simulationStart;
    Duration tick_duration;
    final int targetFps;

    double tickDurationSec;
    long tickDurationNano;
    final double frameDurationSec;
    final long frameDurationNano;
    final long maxUpdateTimeNano;
    int ticksPerFrame = 0;

    TimeUpdate tu;
    TaskStatus status = null;

    final Timer timer;
    final int timerDelta;
    private boolean playing = false;
    private boolean done = false;
    // long lastFrameTime = 0;
    // long lastUpdate = 0;
    // long lastOvershoot = 0;
    //Instant simTime = Instant.EPOCH;

    /*
     * Setup the Control Component
     */

    public Control(Mode mode, Instant simulationStart, SimulationRunner runner, Duration tick_duration, int targetFps, int minFps) {
        this.mode = mode;
        this.runner = runner;
        this.targetFps = targetFps;
        this.counter = new FrameCounter(targetFps);
        frameDurationSec = 1.0 / targetFps;
        frameDurationNano = (Time.SECOND_TO_NANOSEC) / targetFps;
        long maxFrameDuration = (Time.SECOND_TO_NANOSEC) / minFps;
        maxUpdateTimeNano = maxFrameDuration;


        timerDelta = (int) (frameDurationSec * 1000);
        timer = new Timer(timerDelta, this);
        timer.setCoalesce(true);


        setupLayout();

        setTickSpeed(tick_duration, simulationStart);

        updateTimeLabel();
    }

    void setSimulationSpeed(double speed) {
        this.simulationSpeed = speed;
        double deltaPerFrame = speed * frameDurationSec;
        this.ticksPerFrame = (int) Math.floor(deltaPerFrame / tickDurationSec);
    }

    public void init(Duration tick_duration, Instant simulationStart) {
        setTickSpeed(tick_duration, simulationStart);
        done = false;
        playing = false;
        updateTimeLabel();
        statusLabel.setText("INITIALIZED");
        statusLabel.setForeground(Color.BLACK);
        playPauseButton.setIcon(playIcon);
        timer.stop();
    }

    void setTickSpeed(Duration tick_duration, Instant simulationStart) {
        this.simulationStart = simulationStart;
        this.tick_duration = tick_duration;
        tickDurationSec = Time.secondsFromDuration(tick_duration);
        tickDurationNano = tick_duration.getNano();
        tu = new TimeUpdate(simulationStart, tick_duration);
        lastSimTime = tu.oldTime;
        updateDeltaTLabel(tickDurationNano);
        setSimulationSpeed(this.simulationSpeed);
    }


    /*
     * Simulation Control
     */
    long callTime;
    long lastFrameDuration;
    //long nextCallTime;
    long lastMsgTime = 0;
    Instant lastSimTime;

    private void updateSimulation() {
        if (done) return;
        //nextCallTime += timerDelta*1000000;
        long updateStart = System.nanoTime();

        //if (updateStart > nextCallTime) return; // Timer calls catch

        lastFrameDuration = updateStart - callTime;
        callTime = updateStart;

        Duration lastUpdateDuration = Duration.between(lastSimTime, tu.newTime);
        lastSimTime = tu.newTime;

        updateSimSpeedLabel(Time.secondsFromDuration(lastUpdateDuration) / (lastFrameDuration * Time.NANOSEC_TO_SEC));
        addFrame(callTime);

        // if (updateStart - lastMsgTime > Time.SECOND_TO_NANOSEC){
        //     lastMsgTime = updateStart;
        //     long expectedCallTime = nextCallTime-frameDurationNano;            
        //     System.out.println("Current deviation: " + Double.toString((updateStart - expectedCallTime)*0.000001) +"ms");
        // }

        if (fullspeed) {
            updateSimulationFullspeed();
        } else {
            updateSimulationRealtime();
        }

        // Render
        updateTimeLabel();
        runner.redraw();
    }

    private void updateSimulationFullspeed() {
        long t;
        long minTime = callTime + frameDurationNano / 2; // Make sure at least half the time is spent simulating (vs rendering)
        long targetTime = callTime + frameDurationNano;
        do {
            runner.update(tu);
            if (checkStatus(runner.status())) return;

            tu = new TimeUpdate(tu.newTime, tick_duration);
            t = System.nanoTime();
        } while (t < minTime || System.nanoTime() < targetTime);
    }

    private void updateSimulationRealtime() {
        for (int i = 0; i < ticksPerFrame; ++i) {
            runner.update(tu);
            if (checkStatus(runner.status())) return;

            tu = new TimeUpdate(tu.newTime, tick_duration);
            if (System.nanoTime() - callTime > maxUpdateTimeNano) break; // Frame-skip
        }
    }

    boolean checkStatus(TaskStatus status) {
        this.status = status;
        if (status == TaskStatus.RUNNING) return false;

        done = true;
        pause();
        if (status == TaskStatus.SUCCEEDED) {
            statusLabel.setText("SUCCEEDED");
            statusLabel.setForeground(SUCCEEDED_COLOR);
        } else if (status == TaskStatus.FAILED) {
            statusLabel.setText("FAILED");
            statusLabel.setForeground(FAILED_COLOR);
        } else if (status == TaskStatus.FAILED_COLLISION) {
            statusLabel.setText("FAILED: COLLISION");
            statusLabel.setForeground(FAILED_COLOR);
        } else if (status == TaskStatus.FAILED_TIMEOUT) {
            statusLabel.setText("FAILED: TIMEOUT");
            statusLabel.setForeground(FAILED_COLOR);
        }

        return true;
    }

    public void play() {
        if (playing || done) return;

        statusLabel.setText("RUNNING");
        statusLabel.setForeground(RUNNING_COLOR);
        playing = true;
        playPauseButton.setIcon(pauseIcon);
        //System.out.println("Play");
        timer.start();
        callTime = System.nanoTime();
        //nextCallTime = callTime + frameDurationNano;
        // lastFrameTime = lastUpdate;
        // lastFPSTime = lastUpdate;
        // lastOvershoot = 0;
    }

    public void pause() {
        if (!playing) return;
        if (!done) {
            statusLabel.setText("PAUSED");
            statusLabel.setForeground(Color.ORANGE);
        }
        playing = false;
        playPauseButton.setIcon(playIcon);
        //System.out.println("Pause");
        timer.stop();
    }

    private void reset() {
        runner.reset();
        done = false;
        tu = new TimeUpdate(simulationStart, tick_duration);
        lastSimTime = tu.oldTime;
        updateTimeLabel();
        statusLabel.setText("INITIALIZED");
        statusLabel.setForeground(Color.BLACK);
        runner.redraw();
    }


    private String printTime(Instant t) {
        return timeFormatter.format(t);
    }

    private void updateTimeLabel() {
        timeLabel.setText(printTime(tu.oldTime));
    }

    private void updateDeltaTLabel(long deltaT) {
        deltaTLabel.setText(Long.toString(deltaT / 1000000) + "ms");
    }

    private void updateSimSpeedLabel(double speedRatio) {
        simSpeedLabel.setText(ratioFormatter.format(speedRatio));
    }

    private void addFrame(long callTime) {
        counter.addFrame(callTime);
        long avg = counter.getAvgDelta();
        if (avg != 0) {
            double fps = counter.getFramesPerSeconds();
            fpsLabel.setText(ratioFormatter.format(fps));
            // frameDurationLabel.setText(Long.toString(avg/1000000)+"ms");
            // long var = Math.round(Math.sqrt(counter.getVariance()));
            // frameVarianceLabel.setText(Long.toString(var/1000)+"us");
        }
    }


    /*
        UI
    */


    /*
     * Handle the Component Actions
     */

    @Override
    public void actionPerformed(ActionEvent e) {
        Object s = e.getSource();
        if (s == timer) {
            if (!playing) return;
            if (mode == Mode.SIMULATION) {
                updateSimulation();
            } else {
                // TODO
            }
        } else if (s == simModeButton) {
            //System.out.println("Sim Mode: " + ((SimulationMode) simModeButton.getSelectedItem()).name);
            if (((SimulationMode) simModeButton.getSelectedItem()) == SimulationMode.FULL_SPEED) {
                if (!fullspeed) {
                    fullspeed = true;
                    this.remove(1);
                    this.revalidate();
                }
            } else {
                if (fullspeed) {
                    fullspeed = false;
                    this.add(speedButton, 1);
                    this.revalidate();
                }
            }
        } else if (s == speedButton) {
            //System.out.println("Speed: " + ((SimulationSpeed) speedButton.getSelectedItem()).name);
            setSimulationSpeed(((SimulationSpeed) speedButton.getSelectedItem()).factor);
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
        if (e.getSource() == slider) {
            //System.out.println("Slider: "+slider.getValue());
            if (!slider.getValueIsAdjusting()) {
                // TODO
            }
        }
    }

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
                    .withLocale(Locale.UK)
                    .withZone(ZoneId.systemDefault());

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

    private final static SimulationMode simModes[] = {SimulationMode.REAL_TIME, SimulationMode.FULL_SPEED};
    private final static SimulationSpeed simSpeeds[] = {SimulationSpeed.TIMES16, SimulationSpeed.TIMES8,
            SimulationSpeed.TIMES4, SimulationSpeed.TIMES2, SimulationSpeed.REAL_TIME, SimulationSpeed.HALF,
            SimulationSpeed.FOURTH, SimulationSpeed.EIGHTH, SimulationSpeed.SIXTEENTH, SimulationSpeed.DIV50,
            SimulationSpeed.DIV100};

    /*
     * SWING COMPONENTS
     */

    JButton resetButton;
    JComboBox<SimulationMode> simModeButton;
    JComboBox<SimulationSpeed> speedButton;

    JButton stepBackButton;
    JButton playPauseButton;
    JButton stepForwardButton;

    JLabel timeLabel;
    //https://docs.oracle.com/javase/tutorial/uiswing/components/slider.html
    JSlider slider;
    JLabel deltaTLabel;
    JLabel simSpeedLabel;

    JLabel fpsLabel;
    JLabel statusLabel;
    // final JLabel frameDurationLabel;
    // final JLabel frameVarianceLabel;

    // final JLabel manualFPSLabel;


    private void setupLayout() {
        FlowLayout layout = new FlowLayout();
        layout.setAlignment(FlowLayout.LEFT);
        setLayout(layout);
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
        timeLabel = new JLabel();
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

        add(new JLabel("Status: "));
        statusLabel = new JLabel("INITIALIZED");
        add(statusLabel);

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

    public FrameCounter(int trackedFrames) {
        this.trackedFrames = trackedFrames;
        this.deltas = new long[trackedFrames];
    }

    public void addFrame(long timestamp) {
        if (count == 0) {
            lastTime = timestamp;
            count = 1;
            return;
        }
        long delta = timestamp - lastTime;
        if (count == trackedFrames) {
            long last = deltas[currentPos];
            sum += delta - last;
            sumSquared += (delta * delta) - (last * last);
        } else {
            ++count;
            sum += delta;
            sumSquared += (delta * delta);
        }
        deltas[currentPos] = delta;
        lastTime = timestamp;
        currentPos = (currentPos + 1) % trackedFrames;
    }

    public long getAvgDelta() {
        if (count == 0) return 0;
        else return sum / count;
    }

    public long getVariance() {
        if (count == 0) return 0;
        long avg = getAvgDelta();
        return sumSquared / count - avg * avg;
    }

    public double getFramesPerSeconds() {
        long a = getAvgDelta();
        if (a == 0) return 0;
        return Time.SECOND_TO_NANOSEC / (double) a;
    }

}