package de.gdl.rl.environment.games.snimmt;

import de.gdl.rl.agents.Agent;
import de.gdl.rl.agents.RosAgent;
import de.gdl.rl.agents.RosTrainingAgent;

import de.gdl.rl.environment.GDLGameEnvironment;
import de.monticore.lang.gdl.Command;
import de.monticore.lang.gdl.InterpreterOptions;
import de.monticore.lang.gdl.types.GDLNumber;
import de.monticore.lang.gdl.types.GDLTuple;
import de.monticore.lang.gdl.types.GDLType;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;


public class SnimmtEnv extends GDLGameEnvironment {
    private HashMap<GDLType, Integer> numberOfPossibleActionsForRole = new HashMap<GDLType, Integer>();
    private RosTrainingAgent trainingAgent;
    
    //-----------Constructor-------------
    public SnimmtEnv() {
        super(new InterpreterOptions().debugMode(false));
        this.trainingAgent = new RosTrainingAgent()
            .withName("Training Agent")
            .withType("DQN")
            .withGdlRoles(new GDLType[]{GDLType.createFromLine("red"), GDLType.createFromLine("blue"), GDLType.createFromLine("green"), GDLType.createFromLine("yellow")})
            .withGameOverForIllegalActions(false)
            .withCurrentGdlRole(GDLType.createFromLine("red"))
            .withStateTopic("/gdl/6nimmt/trainingAgent/state")
            .withActionTopic("/gdl/6nimmt/trainingAgent/action")
            .withTerminalTopic("/gdl/6nimmt/trainingAgent/terminal")
            .withRewardTopic("/gdl/6nimmt/trainingAgent/reward")
            .withResetTopic("/gdl/6nimmt/trainingAgent/reset");
            
        RosAgent agent = new RosAgent()
            .withName("Agent")
            .withType("DQN")
            .withGdlRoles(new GDLType[]{GDLType.createFromLine("red"), GDLType.createFromLine("blue"), GDLType.createFromLine("green"), GDLType.createFromLine("yellow")})
            .withStateTopic("/gdl/6nimmt/agent/state")
            .withLegalActionsTopic("/gdl/6nimmt/agent/legal_actions")
            .withActionTopic("/gdl/6nimmt/agent/action");

        RosAgent selfPlayAgent = agent
                                .copy()
                                .withName("Self-play Agent")
                                .withNumberOfRandomEpisodes(5000)
                                .withEpsilon(0.9f)
                                .withEpsilonDecay(0.9f);
        
        this.addToTrainingConfiguration(this.trainingAgent);
        this.addToTrainingConfiguration(selfPlayAgent);

        this.addToGamingConfiguration(agent);

        // 20 targets (cannot put cards at pos 1 in stacks 4 stacks with 5 positions to put = 20) + 10 cards in hand + Noop= 31 actions
        this.numberOfPossibleActionsForRole.put(GDLType.createFromLine("red"), 31);
        this.numberOfPossibleActionsForRole.put(GDLType.createFromLine("blue"), 31);
        this.numberOfPossibleActionsForRole.put(GDLType.createFromLine("green"), 31);
        this.numberOfPossibleActionsForRole.put(GDLType.createFromLine("yellow"), 31);


    }
    
    //--------------GDL Model-------------------------------
    protected String getPathToGdlModel() {
        return "src/main/resources/gdl/games/6nimmt.gdl";
    }

    //----------------Zähle mögliche Züge--------------------
    public int getNumberOfActionsForRole(GDLType gdlRole)  {
        return this.numberOfPossibleActionsForRole.get(gdlRole);
    }

    private int episode;
    private int won, lost, second, illegal, other, /*legal,*/ steps;


    //---------------------Verändere Rollen vor Episodenneustart-----------------------
    public void onNextEpisode() {
        // can be used to change things before the next episode starts
        episode++;
        //legal++;
        if(wasLastMoveIllegal()){
            illegal++;
        }else if(!isTerminal()){
            other++;
            //legal++;
        }
        System.out.printf("Episode: %d. Last 50: (Won: %d    Lost: %d    Second Place: %d    Illegal: %d    Other: %d)\n", episode, won, lost, second, illegal, other);
        if (episode % 50 == 0) {
            won = lost = second = illegal = other = 0;
        }
        //rotate roles
        if (this.trainingAgent.currentGdlRole == GDLType.createFromLine("red")) {
            this.trainingAgent.currentGdlRole = GDLType.createFromLine("blue");
        } else if (this.trainingAgent.currentGdlRole == GDLType.createFromLine("blue")) {
            this.trainingAgent.currentGdlRole = GDLType.createFromLine("green");
        } else if (this.trainingAgent.currentGdlRole == GDLType.createFromLine("green")) {
            this.trainingAgent.currentGdlRole = GDLType.createFromLine("yellow");
        } else if (this.trainingAgent.currentGdlRole == GDLType.createFromLine("yellow")) {
            this.trainingAgent.currentGdlRole = GDLType.createFromLine("red");
        }

    }

    //----------Liest Karte auf einem Feld, 0 für leer sonst Zahl auf der Karte--------------------
    private int getFieldStatus(String status) {
        try {
            int num = Integer.parseInt(status);
            return (num == 1000 ? 0 : num);
        }
        catch (NumberFormatException e)
        {
            e.printStackTrace();
            return 0;
        }
    }

    // get state as float representation
    
    public float[] getStateAsFloatRepresentation(GDLType gdlRoleName, Agent agent) {
        
        // (104 cards + 1 blank) * (6*4 spaces on stacks + 10 spaces in hands + 1 card in Play) = 3675
        float[] output = new float[(104+1)*(6*4+10+1)];
        Set<GDLType> state = this.getCurrentStateByRole(gdlRoleName);
        for (GDLType type : state) {
            GDLTuple tuple = (GDLTuple)type;
            if (tuple.get(0).toString().equals("stack")) {
                int slot = Integer.parseInt(tuple.get(2).toString()) - 1;
                int stack = "abcd".indexOf(tuple.get(1).toString());
                
                //Difference between Stacks = 6 slots with 105 bits each, Difference between Slots = 105
                output[stack * (6*105) + slot * 105 + this.getFieldStatus(tuple.get(3).toString())] = 1.0f;
            } else if (tuple.get(0).toString().equals("hand")) {
                int slot = Integer.parseInt(tuple.get(2).toString()) - 1;
                output[4*6*105-1 + slot * 105 + this.getFieldStatus(tuple.get(3).toString())] = 1.0f;
            } else if (tuple.get(0).toString().equals("inPlay")) {
                output[4*6*105-1 + 10*105 + this.getFieldStatus(tuple.get(2).toString())] = 1.0f;
            }
        }
        return output;
    }


    //Map:  Number of Slots on Stack: 5*4 = 20
    //      Play = 10 slots in hand
    //      Noop
    //      Noop = 0
    //      put = 1-20
    //      play = 21-30
    //convert action from agent to mapped move
    public Command getMoveFromAction(int action, GDLType gdlRoleName, Agent agent) {
        steps++;
        String role = gdlRoleName.toString();
        StringBuilder move = new StringBuilder();
        if(action == 0)
        {
            move.append("(noop)");
        } else if(action>20){
            move.append("(play");
            move.append(" ");
            move.append(Integer.toString(action - 20));
            move.append(")");
        } else {
            move.append("(put");
            move.append(" ");
            int stacknum = ((action-1) - (action-1)%5)/5;
            switch (stacknum){
                case 0:
                    move.append("a");
                    break;
                case 1:
                    move.append("b");
                    break;
                case 2:
                    move.append("c");
                    break;
                case 3:
                    move.append("d");
                    break;
                default:
            }
            int slot =  (action-1)%5;
            move.append(" ");
            move.append(Integer.toString(slot+2));
            move.append(")");
        }

        return Command.createFromLine(role + " " + move.toString());
    }

    //TODO moveString Splitted mit oder ohne Klammern?
    public int getActionFromMove(Command move, Agent agent) {
        //TODO Rewrite for short commands
        //<role> (play <pos>) pos=>[1:10]
        //<role> (put <stack> <pos>) stack => [a:d], pos => [1:6]
        
        GDLTuple moveTuple = (GDLTuple)move.getAction();
        if(moveTuple.get(0).toString().equals("noop")){
            return 0;
        }
        if(moveTuple.get(0).toString().equals("play")){
            return (20+Integer.parseInt(moveTuple.get(1).toString()));
        } else {
            int targetStack = "abcd".indexOf(moveTuple.get(1).toString());
            //Only slots 2-6 can be chosen
            int targetSlot = Integer.parseInt(moveTuple.get(2).toString()) - 1;
            return targetStack*5+targetSlot;
        }
    }

    // calculate the reward for specific role
    // Für jeden Schritt
    //--------------------------------Reward output nach Goal, negativer Reward für illegal move-------------------------
    public float calculateRewardFromGoals(Map<GDLType,GDLType> goals, GDLType gdlRoleName, Agent agent) {
        if(this.wasLastMoveIllegal() && this.whichRolesHaveControl().contains(gdlRoleName)){
            illegal++;
            float reward = -10.0f+(((float)steps)*0.25f);
            steps = 0;
            return reward;
        } else if(!this.isTerminal()){
            other++;
            return 0.0f;
        } else if(!this.wasLastMoveIllegal() && this.isTerminal()){
            int goal = ((GDLNumber) goals.get(gdlRoleName)).getValue().intValue();
            if (goal >= 100) {
                won++;
                return 10.0f;
            } else if (goal >= 50){
                second++;
                return 5.0f;
            } else {
                lost++;
                return -7.5f;
            }
        }
        System.out.println("Unexpected End");
        return 0.0f; 
    }

//====================================TODO=================================================

    //Get padded numbers 'XXX' ' XX' ' X '
    public String getStringSlotStatus(String status){
        int card = Integer.parseInt(status);
        String output;
        if(card<10){
            output = " "+card+" ";
        } else if(card<100){
            output = " "+card;
        } else{
            output = Integer.toString(card);
        }
        return output;
    }
    
    public String getStateAsReadableString() {
        // A B C D Stacks
        // R U G Y Player Hands
        // r u g y played Cards
        // 0 = empty

        //  A 0 0 0 0 0 0
        //  B 0 0 0 0 0 0
        //  C 0 0 0 0 0 0
        //  D 0 0 0 0 0 0
        // -------------------------------
        //  r-0 u-0 g-0 y-0
        // -------------------------------
        //  R 0 0 0 0 0 0 0 0 0 0
        //  U 0 0 0 0 0 0 0 0 0 0
        //  G 0 0 0 0 0 0 0 0 0 0
        //  Y 0 0 0 0 0 0 0 0 0 0
        //  

        //0-23 Stacks, 24-27 inPLay, 28-67 hands
        String[] slotStates = new String[6*4+4+4*10];
        String[] points = new String[4];
        Map<GDLType, Set<GDLType>> hiddenState = this.getCurrentHiddenState(); //inPLay, hands
        Set<GDLType> visibleState = this.getCurrentState(); //stacks, points
        for (GDLType type : visibleState) {
            GDLTuple tuple = (GDLTuple)type;
            if (tuple.get(0).toString().equals("stack")) {
                int slot = Integer.parseInt(tuple.get(2).toString()) - 1;
                int stack = "abcd".indexOf(tuple.get(1).toString());
                //0-23
                slotStates[stack*6 + slot] = this.getStringSlotStatus(tuple.get(3).toString());
            } else if (tuple.get(0).toString().equals("currPoints")){
                if (tuple.get(1).toString().equals("red")){
                    points[0] = tuple.get(2).toString();
                } else if (tuple.get(1).toString().equals("blue")){
                    points[1] = tuple.get(2).toString();
                } else if (tuple.get(1).toString().equals("green")){
                    points[2] = tuple.get(2).toString();
                } else if (tuple.get(1).toString().equals("yellow")){
                    points[3] = tuple.get(2).toString();
                }
            }
        }
        // red = 0, blue = 1, green = 2, yellow = 3
        for (GDLType type : hiddenState.get(GDLType.createFromLine("red"))){
            GDLTuple tuple = (GDLTuple)type;
            if(tuple.get(0).toString().equals("inPlay")){
                //24
                slotStates[24+0] = this.getStringSlotStatus(tuple.get(2).toString());
            } else if(tuple.get(0).toString().equals("hand")){
                int slot = Integer.parseInt(tuple.get(2).toString()) - 1;
                //28-37
                slotStates[28+0*10+slot] = this.getStringSlotStatus(tuple.get(3).toString());
            }
        }
        for (GDLType type : hiddenState.get(GDLType.createFromLine("blue"))){
            GDLTuple tuple = (GDLTuple)type;
            if(tuple.get(0).toString().equals("inPlay")){
                //25
                slotStates[24+1] = this.getStringSlotStatus(tuple.get(2).toString());
            } else if(tuple.get(0).toString().equals("hand")){
                int slot = Integer.parseInt(tuple.get(2).toString()) - 1;
                //38-47
                slotStates[28+1*10+slot] = this.getStringSlotStatus(tuple.get(3).toString());
            }
        }
        for (GDLType type : hiddenState.get(GDLType.createFromLine("green"))){
            GDLTuple tuple = (GDLTuple)type;
            if(tuple.get(0).toString().equals("inPlay")){
                //26
                slotStates[24+2] = this.getStringSlotStatus(tuple.get(2).toString());
            } else if(tuple.get(0).toString().equals("hand")){
                int slot = Integer.parseInt(tuple.get(2).toString()) - 1;
                //48-57
                slotStates[28+2*10+slot] = this.getStringSlotStatus(tuple.get(3).toString());
            }
        }
        for (GDLType type : hiddenState.get(GDLType.createFromLine("yellow"))){
            GDLTuple tuple = (GDLTuple)type;
            if(tuple.get(0).toString().equals("inPlay")){
                //27
                slotStates[24+3] = this.getStringSlotStatus(tuple.get(2).toString());
            } else if(tuple.get(0).toString().equals("hand")){
                int slot = Integer.parseInt(tuple.get(2).toString()) - 1;
                //58-67
                slotStates[28+3*10+slot] = this.getStringSlotStatus(tuple.get(3).toString());
            }
        }
        StringBuilder readableStringBuilder = new StringBuilder();
        readableStringBuilder.append("=======Stacks========\n");
        readableStringBuilder.append("  A    B    C    D  \n");
        
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 4; j++) {
                //0-23
                readableStringBuilder.append(" " + slotStates[j*6+i] + " ");
            }
            readableStringBuilder.append("\n");
        }
        readableStringBuilder.append("====Cards Played ====\n");
        readableStringBuilder.append("  R    B    G    Y  \n");
        for (int k = 0; k < 4; k++) {
            //24-27
            readableStringBuilder.append(" " + slotStates[24+k] + " ");
        }
        readableStringBuilder.append("\n");
        readableStringBuilder.append("========Hands========\n");
        readableStringBuilder.append("  R    B    G    Y  \n");
        for (int l = 0; l < 10; l++) {
            for (int m = 0; m < 4; m++) {
                //28-67
                readableStringBuilder.append(" " + slotStates[24+4+m*10+l] + " ");
            }
            readableStringBuilder.append("\n");
        }
        readableStringBuilder.append("\n");
        readableStringBuilder.append("========Points========\n");
        readableStringBuilder.append("  R    B    G    Y  \n");
        for (int n = 0; n<4; n++) {
            //remove whitespace when points reach 2 or 3 digits to keep format
            if(points[n].length() <= 1){
                readableStringBuilder.append("  " + points[n] + "  ");
            } else if(points[n].length() == 2){
                readableStringBuilder.append("  " + points[n] + " ");
            } else {
                readableStringBuilder.append(" " + points[n] + " ");
            }
            
        }
        
        return readableStringBuilder.toString();
    }
    
    public static void main(String[] args) throws Exception {
        GDLGameEnvironment.initEnvironment(new SnimmtEnv(), args);
    }
}

/* Float Representation
Einheit [----] = 105 

State Stack
Stack A 1 [-----] (0-104)
Stack A 2 [-----] (105-209)
Stack A 3 [-----] (210-314)
Stack A 4 [-----] (315-419)
Stack A 5 [-----] (420-524)
Stack A 6 [-----] (525-629)

Stack B 1 [-----] (630-734)
Stack B 2 [-----] (735-839)
Stack B 3 [-----] (840-944)
Stack B 4 [-----] (945-1049)
Stack B 5 [-----] (1050-1154)
Stack B 6 [-----] (1155-1259)

Stack C 1 [-----] (1260-1364)
Stack C 2 [-----] (1365-1469)
Stack C 3 [-----] (1470-1574)
Stack C 4 [-----] (1575-1679)
Stack C 5 [-----] (1680-1784)
Stack C 6 [-----] (1785-1889)

Stack D 1 [-----] (1890-1994)
Stack D 2 [-----] (1995-2099)
Stack D 3 [-----] (2100-2204)
Stack D 4 [-----] (2205-2309)
Stack D 5 [-----] (2310-2414)
Stack D 6 [-----] (2415-2519)

Hand 1  [-----] (2520-2624)
Hand 2  [-----] (2625-2729)
Hand 3  [-----] (2730-2834)
Hand 4  [-----] (2835-2939)
Hand 5  [-----] (2940-3044)
Hand 6  [-----] (3045-3149)
Hand 7  [-----] (3150-3254)
Hand 8  [-----] (3255-3359)
Hand 9  [-----] (3360-3464)
Hand 10  [-----] (3465-3569)

InPlay [-----] (3570-3674)
*/

/* move |   Action
noop	|	0
put a 2	|	1
put a 3	|	2
put a 4	|	3
put a 5	|	4
put a 6	|	5
put b 2	|	6
put b 3	|	7
put b 4	|	8
put b 5	|	9
put b 6	|	10
put c 2	|	11
put c 3	|	12
put c 4	|	13
put c 5	|	14
put c 6	|	15
put d 2	|	16
put d 3	|	17
put d 4	|	18
put d 5	|	19
put d 6	|	20
play 1	|	21
play 2	|	22
play 3	|	23
play 4	|	24
play 5	|	25
play 6	|	26
play 7	|	27
play 8	|	28
play 9	|	29
play 10	|	30*/