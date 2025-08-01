package de.gdl.rl.environment.games.doppelkopf;

import static org.junit.Assert.assertEquals;

import org.junit.Test;


public class MappingTest {

    @Test
    public void stateMappingTest() {
        DoppelkopfEnvironment.stateMappingTest(new DoppelkopfEnvironment(), 2);
    }

}
