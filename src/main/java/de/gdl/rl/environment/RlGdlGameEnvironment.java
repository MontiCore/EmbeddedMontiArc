package de.gdl.rl.environment;
import java.util.List;
public interface RlGdlGameEnvironment {
    public float[] getStateAsFloatRepresentation(String gdlRoleName);
    public String getMoveStringFromAction(int action, String gdlRoleName);
    public float calculateRewardFromGoals(List<List<String>> goals, String gdlRoleName);
    public float[] getLegalActionsForPlayerAsIndicatorArray(String gdlRoleName);
}
