package de.gdl.rl.environment;
import java.util.List;

import de.gdl.rl.agents.Agent;
public interface RlGdlGameEnvironment {
    /**
    * Converts the current state for an agent and a role 
    into a vector encoded as a float array.
    * @param gdlRoleName the role for which the state is converted
    * @param agent the agent for which the state is converted
    * @return a float array that describes the current state for an agent and a role
    */
    public float[] getStateAsFloatRepresentation(String gdlRoleName, Agent agent);
    
    /**
    * Maps an action encoded as a number of an agent for a role to a move
    * @param action the action that the agent performs
    * @param gdlRoleName The role to which the action belongs
    * @param agent the agent that performs the action
    * @return the agent's move as a string
    */
    public String getMoveStringFromAction(int action, String gdlRoleName, Agent agent);
    
    /**
    * Maps a move back to the corresponding number of the action for a role of an agent 
    * @param moveStringSplitted the tuple as a list, which the move describes (without role name)
    * @param gdlRoleName The role belonging to the move
    * @param agent the corresponding agent for which the numbering is
    * @return the number of the agent's action for the given role
    */
    public int getActionFromMoveString(List<String> moveStringSplitted, String gdlRoleName, Agent agent);
    
    /**
     * States how many different actions a role has in total
     * @param roleName the corresponding role name
     * @return the number of different actions that exist for this role
     */
    public int getNumberOfActionsForRole(String roleName);
}
