package de.monticore.lang.gdl.doppelkopf;

import java.util.Map;

import de.monticore.lang.gdl.Interpreter;

public class DoppelkopfGUI {

    private Interpreter interpreter;
    private Map<String, PlayerFrame> playerFrames;

    public DoppelkopfGUI(Interpreter interpreter) {
        this.interpreter = interpreter;
        initPlayerFrames();
    }

    private void initPlayerFrames() {
        playerFrames = Map.of(
            "player1", new PlayerFrame("player1", interpreter),
            "player2", new PlayerFrame("player2", interpreter),
            "player3", new PlayerFrame("player3", interpreter),
            "player4", new PlayerFrame("player4", interpreter)
        );

        playerFrames.get("player1").setLocation(100, 100);
        playerFrames.get("player2").setLocation(
            playerFrames.get("player1").getWidth() + 200,
            100
        );
        playerFrames.get("player3").setLocation(
            100,
            playerFrames.get("player1").getHeight() + 300
        );
        playerFrames.get("player4").setLocation(
            playerFrames.get("player1").getWidth() + 200,
            playerFrames.get("player1").getHeight() + 300
        );
    }
    
}
