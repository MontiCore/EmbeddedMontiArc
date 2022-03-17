package de.gdl.rl.agents;
import java.util.List;
import java.util.HashSet;

public class LocalAgent {
    public String name = "LocalAgent";    
    public HashSet<String> gdlRoleNames = new HashSet<String>();
    public boolean gameOverForIllegalActions = false;
    public String getMove(List<List<String>> state, List<String> legalMoves, String role, int numberOfEpisodesPlayed) {
        return "";
    }
}
