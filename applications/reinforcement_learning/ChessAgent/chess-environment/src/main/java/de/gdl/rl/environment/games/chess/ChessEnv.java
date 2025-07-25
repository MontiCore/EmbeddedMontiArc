package de.gdl.rl.environment.games.chess;

import de.gdl.rl.agents.Agent;
import de.gdl.rl.agents.RosAgent;
import de.gdl.rl.agents.RosTrainingAgent;

import de.gdl.rl.environment.GDLGameEnvironment;

import java.util.HashMap;
import java.util.List;


public class ChessEnv extends GDLGameEnvironment {
    
    private HashMap<String, Integer> numberOfPossibleActionsForRole = new HashMap<String, Integer>();
    private RosTrainingAgent trainingAgent;
    
    public ChessEnv() {
        
        this.trainingAgent = new RosTrainingAgent()
            .withName("Training Agent")
            .withType("DQN")
            .withGdlRoleNames(new String[]{"white", "black"})
            .withGameOverForIllegalActions(true)
            .withCurrentGdlRoleName("x")
            .withStateTopic("/gdl/chess/trainingAgent/state")
            .withActionTopic("/gdl/chess/trainingAgent/action")
            .withTerminalTopic("/gdl/chess/trainingAgent/terminal")
            .withRewardTopic("/gdl/chess/trainingAgent/reward")
            .withResetTopic("/gdl/chess/trainingAgent/reset");
            
        RosAgent agent = new RosAgent()
            .withName("Agent")
            .withType("DQN")
            .withGdlRoleNames(new String[]{"white", "black"})
            .withStateTopic("/gdl/chess/agent/state")
            .withLegalActionsTopic("/gdl/chess/agent/legal_actions")
            .withActionTopic("/gdl/chess/agent/action");

        RosAgent selfPlayAgent = agent
                                .copy()
                                .withName("Self-play Agent")
                                .withNumberOfRandomEpisodes(50)
                                .withEpsilon(0.9f)
                                .withEpsilonDecay(0.9f);
        
        this.addToTrainingConfiguration(this.trainingAgent);
        this.addToTrainingConfiguration(selfPlayAgent);

        this.addToGamingConfiguration(agent);

        this.numberOfPossibleActionsForRole.put("white", 4096);
        this.numberOfPossibleActionsForRole.put("black", 4096);


    }

    protected String getPathToGdlModel() {
        return "src/main/resources/gdl/games/Chess.gdl";
    }

    public int getNumberOfActionsForRole(String roleName)  {
        return this.numberOfPossibleActionsForRole.get(roleName);
    }

    public void onNextEpisode() {
        // can be used to change things before the next episode starts

        if (this.trainingAgent.currentGdlRoleName == "white") {
            this.trainingAgent.currentGdlRoleName = "black";
        } else {
            this.trainingAgent.currentGdlRoleName = "white";
        }

    }

    private int getFieldStatus(String status, boolean inverted) {
        String infix = status.substring(0, 5);
        String figure = null;
        int offset = !inverted ? 0 : 6;
        if (infix.equals("white")) {
            figure = "white";
        } else if(infix.equals("black")) {
            figure = "black";
            offset = !inverted ? 6 : 0;;
        } 
        if (figure != null) {
            String suffix = status.substring(6);
            
            switch (suffix) {
                case "pawn":
                    return 0 + offset;
                case "rook":
                    return 1 + offset;
                case "knight":
                    return 2 + offset;
                case "bishop":
                    return 3 + offset;
                case "queen":
                    return 4 + offset;
                case "king":
                    return 5 + offset;
                default:
                    break;
            }

        }

        return 12;
    }

    // get state as float representation
    public float[] getStateAsFloatRepresentation(String gdlRoleName, Agent agent) {
        
        // (6*2 figures + 1 blank) * (64 fields) = 832 
        // 4 castle, 9 en passamt = 13
        // total = 845
        boolean inverted = gdlRoleName.equals("black");

        float castle_black_long = 1.0f;
        float castle_black_short = 1.0f;
        float castle_white_long = 1.0f;
        float castle_white_short = 1.0f;

        int enPassant = 0;

        float[] output = new float[((6*2 + 1) * 64) + 4 + 9];

        for (List<String> tuple : this.getCurrentState()) {
            if (tuple.get(0).equals("field")) {
                int row = Integer.parseInt(tuple.get(2)) - 1;
                int cell = "abcdefgh".indexOf(tuple.get(1));

                if (inverted) {
                    row = 7 - row;
                    cell = 7 - cell;
                }

                output[row * (8 * 13) + cell * 13 + this.getFieldStatus(tuple.get(3), inverted)] = 1.0f;
            } else if (tuple.get(0).equals("castle")) {
                if (tuple.get(1).equals("black")) {
                    if (tuple.get(2).equals("long") && !tuple.get(3).equals("allowed")) {
                        castle_black_long = 0.0f;
                    } else if(!tuple.get(3).equals("allowed")){
                        castle_black_short = 0.0f;
                    }
                } else {
                    if (tuple.get(2).equals("long") && !tuple.get(3).equals("allowed")) {
                        castle_white_long = 0.0f;
                    } else if(!tuple.get(3).equals("allowed")) {
                        castle_white_short = 0.0f;
                    }
                }
            } else if (tuple.get(0).equals("enPassant")) {
                if (!tuple.get(1).equals("none")) {
                    enPassant = "abcdefgh".indexOf(tuple.get(1)) + 1;
                }
               
            }

        }

        output[((6*2 + 1) * 64)] = inverted ? castle_black_long : castle_white_long;
        output[((6*2 + 1) * 64) + 1] = inverted ? castle_black_short : castle_white_short;
        output[((6*2 + 1) * 64) + 2] = inverted ? castle_white_long : castle_black_long;
        output[((6*2 + 1) * 64) + 3] = inverted ? castle_white_short : castle_black_short;

        output[((6*2 + 1) * 64) + 4 + enPassant] = 1.0f;

        return output;
    }

    // convert action from agent to mapped move
    public String getMoveStringFromAction(int action, String gdlRoleName, Agent agent) {
        
        boolean inverted = gdlRoleName.equals("black");

        int fromField = action / 64;
        int toField = action % 64;

        int fromFieldRow = fromField / 8;
        int fromFieldCell = fromField % 8;

        int toFieldRow = toField / 8;
        int toFieldCell = toField % 8;

        if (inverted) {
            fromFieldRow = 7 - fromFieldRow;
            fromFieldCell = 7 - fromFieldCell;
            toFieldRow = 7 - toFieldRow;
            toFieldCell = 7 - toFieldCell;
        }

        String fromFieldCellLetter = Character.toString("abcdefgh".charAt(fromFieldCell));
        String fromFieldRowLetter = Integer.toString(fromFieldRow + 1);
        String toFieldCellLetter = Character.toString("abcdefgh".charAt(toFieldCell));
        String toFieldRowLetter = Integer.toString(toFieldRow + 1);
        
        String figure = "blank";
        
        for (List<String> tuple : this.getCurrentState()) {
            if (tuple.get(0).equals("field")) {
                //System.out.println(fromFieldCellLetter + " " + tuple.get(1) + " " +fromFieldRowLetter + " " + tuple.get(2));
                if (tuple.get(1).equals(fromFieldCellLetter) && tuple.get(2).equals(fromFieldRowLetter)) {
                    figure = tuple.get(3);
                    break;
                }
            }
        }

        return gdlRoleName + " (move " + figure +" " + fromFieldCellLetter + " " + fromFieldRowLetter+ " " + toFieldCellLetter + " " + toFieldRowLetter + ")";
    }

    // ... and the direction back!
    // used for calculating legal-move-matrix
    public int getActionFromMoveString(List<String> moveStringSplitted, String gdlRoleName, Agent agent) {
        
        boolean inverted = gdlRoleName.equals("black");

        int fromFieldCell = "abcdefgh".indexOf(moveStringSplitted.get(2));
        int fromFieldRow = Integer.parseInt(moveStringSplitted.get(3)) - 1;
        int toFieldCell = "abcdefgh".indexOf(moveStringSplitted.get(4));
        int toFieldRow = Integer.parseInt(moveStringSplitted.get(5)) - 1;


        if (inverted) {
            fromFieldRow = 7 - fromFieldRow;
            fromFieldCell = 7 - fromFieldCell;
            toFieldRow = 7 - toFieldRow;
            toFieldCell = 7 - toFieldCell;
        }

        int fromField = fromFieldRow * 8 * 64 + fromFieldCell * 64;
        int toField = toFieldRow * 8 + toFieldCell;

        return fromField + toField;
    }

    // calculate the reward for specific role
    public float calculateRewardFromGoals(List<List<String>> goals, String gdlRoleName, Agent agent) {
        
        if ((!this.wasLastMoveIllegal() || !this.whichRolesHaveControl().get(0).equals(gdlRoleName)) && this.isTerminal() && goals.size() == 2) {
            if (goals.get(0).get(0) == gdlRoleName) {
                return Integer.parseInt(goals.get(0).get(1)) > 0 ? 10.0f : -1.0f;
            } else {
                return Integer.parseInt(goals.get(1).get(1)) > 0 ? 10.0f : -1.0f;
            }
        }  else if(this.wasLastMoveIllegal() && this.whichRolesHaveControl().get(0).equals(gdlRoleName)) { 
            return -10.0f;
        }

        return 0.0f;

    }

    // CLI-functions

    private char getLetterForFieldStatus(String status) {
        String infix = status.substring(0, 5);
        String figure = null;
        if (infix.equals("white")) {
            figure = "white";
        } else if(infix.equals("black")) {
            figure = "black";
        } 
        if (figure != null) {
            boolean isWhite = infix.equals("white");
            String suffix = status.substring(6);
            switch (suffix) {
                case "pawn":
                    return isWhite ? 'P' : 'p';
                case "rook":
                    return isWhite ? 'R' : 'r';
                case "knight":
                    return isWhite ? 'N' : 'n';
                case "bishop":
                    return isWhite ? 'B' : 'b';
                case "queen":
                    return isWhite ? 'Q' : 'q';
                case "king":
                    return isWhite ? 'K' : 'k';
                default:
                    break;
            }
        }
        return ' ';
    }

    public String getStateAsReadableString() {
        // (K,k) king, (Q,q) queen, (R,r) rook, (B,b) bishop,  N knight, P pawn
        // uppercase: white, lowercase: black

        //        A  B  C  D  E  F  G  H

        //   8    r  n  b  k  q  b  n  e    8  
        //   7    p  p  p  p  p  p  p  p    7
        //   6                              6
        //   5                              5
        //   4                              4
        //   3                              3
        //   2    P  P  P  P  P  P  P  P    2
        //   1    R  N  B  Q  K  B  N  R    1
        
        //        A  B  C  D  E  F  G  H
        //   

        

        char[] fieldStates = new char[64];
        for (List<String> tuple : this.getCurrentState()) {
            if (tuple.get(0).equals("field")) {
                int row = Integer.parseInt(tuple.get(2)) - 1;
                int cell = "abcdefgh".indexOf(tuple.get(1));
                fieldStates[row*8 + cell] = this.getLetterForFieldStatus(tuple.get(3));
            }
        }
        StringBuilder readableStringBuilder = new StringBuilder();
        readableStringBuilder.append("        A  B  C  D  E  F  G  H\n\n");
        
        for (int i = 0; i < 8; i++) {
            int row = 7 - i;
            readableStringBuilder.append("   " + (row + 1) + "    ");
            for (int j = 0; j < 8; j++) {
                readableStringBuilder.append(fieldStates[row*8 + j] + "  ");
            }
            readableStringBuilder.append("  "+ (row + 1) + "\n");
        }
        readableStringBuilder.append("\n");
        readableStringBuilder.append("        A  B  C  D  E  F  G  H\n");
        
        return readableStringBuilder.toString();
    }

    public static void main(String[] args) throws Exception {
        GDLGameEnvironment.initEnvironment(new ChessEnv(), args);
    }

}