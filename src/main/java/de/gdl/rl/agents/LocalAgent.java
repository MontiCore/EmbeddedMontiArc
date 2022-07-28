package de.gdl.rl.agents;
import java.util.List;
import java.util.Set;

import de.monticore.lang.gdl.Command;
import de.monticore.lang.gdl.types.GDLType;

public abstract class LocalAgent extends Agent {
    /** 
     * [override me]
     * Returns a move for a role
     * @param state the current state
     * @param legalMoves a list of all available legal moves for the role
     * @param role the role for which the move is
     * @param numberOfEpisodesPlayed the number of all episodes played since the launch of the environment
     * @return a move to be performed for the specified role
     */
    public abstract Command getMove(Set<GDLType> state, List<Command> legalMoves, GDLType role, int numberOfEpisodesPlayed);
}
