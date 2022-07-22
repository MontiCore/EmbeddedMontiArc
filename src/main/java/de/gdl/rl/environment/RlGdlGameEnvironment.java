package de.gdl.rl.environment;

import de.gdl.rl.agents.Agent;
import de.monticore.lang.gdl.Command;
import de.monticore.lang.gdl.types.GDLType;
public interface RlGdlGameEnvironment {
    /**
    * Converts the current state for an agent and a role 
    into a vector encoded as a float array.
    * @param gdlRole the role for which the state is converted
    * @param agent the agent for which the state is converted
    * @return a float array that describes the current state for an agent and a role
    */
    public float[] getStateAsFloatRepresentation(GDLType gdlRole, Agent agent);
    
    /**
    * Maps an action encoded as a number of an agent for a role to a move
    * @param action the action that the agent performs
    * @param gdlRole The role to which the action belongs
    * @param agent the agent that performs the action
    * @return the agent's move
    */
    public Command getMoveFromAction(int action, GDLType gdlRole, Agent agent);
    
    /**
    * Maps a move back to the corresponding number of the action for a role of an agent 
    * @param move the respective move to be mapped
    * @param agent the corresponding agent for which the numbering is
    * @return the number of the agent's action for the given role
    */
    public int getActionFromMove(Command move, Agent agent);
    
    /**
     * States how many different actions a role has in total
     * @param gdlRole the corresponding role
     * @return the number of different actions that exist for this role
     */
    public int getNumberOfActionsForRole(GDLType gdlRole);
}
