package de.gdl.rl.environment.games.chess;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class MappingTest {
    
    
    @Test
    public void bidirectionalActionMappingTest() {
        ChessEnv.bidirectionalActionMappingTest(new ChessEnv());
    }

    @Test
    public void stateMappingTest() {
        ChessEnv.stateMappingTest(new ChessEnv(), 2);
        
        //assertEquals(true, false);
    }

}
