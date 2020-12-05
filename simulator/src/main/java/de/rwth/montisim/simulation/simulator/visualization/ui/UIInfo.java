/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.ui;

import java.awt.Color;

public class UIInfo {
    public static final double SCROLL_FACTOR = 1.2;
    public static final int MARGIN = 5;
    public static final int LINE_SPACE = 5;
    
    public static final Color TEXT_COLOR = new Color(50, 50, 50);
    public static final Color PANEL_COLOR = new Color(255, 255, 255, 180);
    
    public static boolean antialiasing = false;
    public static boolean inspectAutopilots = true;
    public static boolean drawPlannedTrajectory = true;
    public static boolean drawPlannedPath = true;
    public static boolean drawTargets = true;
    public static boolean drawActuators = true;
    public static boolean showSegments = false;
}