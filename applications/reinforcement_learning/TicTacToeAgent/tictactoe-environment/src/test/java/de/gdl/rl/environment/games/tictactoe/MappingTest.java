package de.gdl.rl.environment.games.tictactoe;

import static org.junit.Assert.assertEquals;

import org.junit.Test;


public class MappingTest {
    
    
    @Test
    public void bidirectionalActionMappingTest() {
        TicTacToeEnv.bidirectionalActionMappingTest(new TicTacToeEnv());
    }

    @Test
    public void stateMappingTest() {
        TicTacToeEnv.stateMappingTest(new TicTacToeEnv(), 2);
        //assertEquals(true, false);
    }

}
