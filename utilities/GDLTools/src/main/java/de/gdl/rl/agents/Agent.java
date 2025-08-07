package de.gdl.rl.agents;

import java.util.HashSet;
import java.util.Set;

import de.monticore.lang.gdl.types.GDLType;

public class Agent {
    
    public String name = "";
    public String type = "";
    public Set<GDLType> gdlRoles = new HashSet<>();
    
    public boolean gameOverForIllegalActions = false;
    public int numberOfRandomEpisodes = 0; // determines how many initial episodes should be played completely randomly
    public float epsilon = 0.0f; // percentage of steps taken completely at random
    public float epsilonDecay = 0.0f; // determines how much epsilon decreases over time

}
