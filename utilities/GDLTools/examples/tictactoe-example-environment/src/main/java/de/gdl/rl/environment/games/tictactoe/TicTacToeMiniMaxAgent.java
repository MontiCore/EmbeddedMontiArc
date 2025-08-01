package de.gdl.rl.environment.games.tictactoe;

import java.util.List;
import java.util.Set;

import org.apache.commons.collections4.BidiMap;
import org.apache.commons.collections4.bidimap.DualHashBidiMap;

import de.gdl.rl.agents.LocalAgent;
import de.monticore.lang.gdl.Command;
import de.monticore.lang.gdl.types.GDLNumber;
import de.monticore.lang.gdl.types.GDLTuple;
import de.monticore.lang.gdl.types.GDLType;
import de.monticore.lang.gdl.types.GDLValue;

public class TicTacToeMiniMaxAgent extends LocalAgent {

    private BidiMap<Integer, String> actionMoveMap = new DualHashBidiMap<Integer, String>();

    public TicTacToeMiniMaxAgent() {
        actionMoveMap.put(0, "(mark 1 1)");
        actionMoveMap.put(1, "(mark 1 2)");
        actionMoveMap.put(2, "(mark 1 3)");

        actionMoveMap.put(3, "(mark 2 1)");
        actionMoveMap.put(4, "(mark 2 2)");
        actionMoveMap.put(5, "(mark 2 3)");

        actionMoveMap.put(6, "(mark 3 1)");
        actionMoveMap.put(7, "(mark 3 2)");
        actionMoveMap.put(8, "(mark 3 3)");
    }

    @Override
    public Command getMove(Set<GDLType> states, List<Command> legalMoves, GDLType role, int numberOfEpisodesPlayed) {
        char[][] board = new char[3][3];

        for (GDLType state : states) {
            if (state instanceof GDLTuple) {
                GDLTuple stateTuple = (GDLTuple) state;
                if (stateTuple.get(0).toString().equals("cell")) {
                    int i = ((GDLNumber) stateTuple.get(1)).getValue().intValue() - 1;
                    int j = ((GDLNumber) stateTuple.get(2)).getValue().intValue() - 1;
                    String value = ((GDLValue) stateTuple.get(3)).getValue();

                    board[i][j] = value.charAt(0) == 'b' ? '_' : value.charAt(0);

                    if (role.toString().equals("o")) {
                        board[i][j] = value.charAt(0) == 'x' ? 'o' : board[i][j];
                        board[i][j] = value.charAt(0) == 'o' ? 'x' : board[i][j];
                    }
                }
            }
        }
    
        TicTacToeAI.Move bestMove = TicTacToeAI.findBestMove(board);

        return Command.createFromLine(role + " (mark " + (bestMove.row + 1) + " " + (bestMove.col + 1) + ")");
    }
    
}
// code taken from here: https://de.acervolima.com/minimax-algorithmus-in-der-spieltheorie-set-3-(tic-tac-toe-ai-den-optimalen-zug-finden)/
class TicTacToeAI
{
    public static class Move
    {
        int row, col;
    };
    
    static char player = 'x', opponent = 'o';
    
    // This function returns true if there are moves
    // remaining on the board. It returns false if
    // there are no moves left to play.
    static Boolean isMovesLeft(char board[][])
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                if (board[i][j] == '_')
                    return true;
        return false;
    }
    
    // This is the evaluation function as discussed
    // in the previous article ( http://goo.gl/sJgv68 )
    static int evaluate(char b[][])
    {
        // Checking for Rows for X or O victory.
        for (int row = 0; row < 3; row++)
        {
            if (b[row][0] == b[row][1] &&
                b[row][1] == b[row][2])
            {
                if (b[row][0] == player)
                    return +10;
                else if (b[row][0] == opponent)
                    return -10;
            }
        }
    
        // Checking for Columns for X or O victory.
        for (int col = 0; col < 3; col++)
        {
            if (b[0][col] == b[1][col] &&
                b[1][col] == b[2][col])
            {
                if (b[0][col] == player)
                    return +10;
    
                else if (b[0][col] == opponent)
                    return -10;
            }
        }
    
        // Checking for Diagonals for X or O victory.
        if (b[0][0] == b[1][1] && b[1][1] == b[2][2])
        {
            if (b[0][0] == player)
                return +10;
            else if (b[0][0] == opponent)
                return -10;
        }
    
        if (b[0][2] == b[1][1] && b[1][1] == b[2][0])
        {
            if (b[0][2] == player)
                return +10;
            else if (b[0][2] == opponent)
                return -10;
        }
    
        // Else if none of them have won then return 0
        return 0;
    }
    
    // This is the minimax function. It considers all
    // the possible ways the game can go and returns
    // the value of the board
    static int minimax(char board[][],
                        int depth, Boolean isMax)
    {
        int score = evaluate(board);
    
        // If Maximizer has won the game
        // return his/her evaluated score
        if (score == 10)
            return score;
    
        // If Minimizer has won the game
        // return his/her evaluated score
        if (score == -10)
            return score;
    
        // If there are no more moves and
        // no winner then it is a tie
        if (isMovesLeft(board) == false)
            return 0;
    
        // If this maximizer's move
        if (isMax)
        {
            int best = -1000;
    
            // Traverse all cells
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    // Check if cell is empty
                    if (board[i][j]=='_')
                    {
                        // Make the move
                        board[i][j] = player;
    
                        // Call minimax recursively and choose
                        // the maximum value
                        best = Math.max(best, minimax(board,
                                        depth + 1, !isMax));
    
                        // Undo the move
                        board[i][j] = '_';
                    }
                }
            }
            return best;
        }
    
        // If this minimizer's move
        else
        {
            int best = 1000;
    
            // Traverse all cells
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    // Check if cell is empty
                    if (board[i][j] == '_')
                    {
                        // Make the move
                        board[i][j] = opponent;
    
                        // Call minimax recursively and choose
                        // the minimum value
                        best = Math.min(best, minimax(board,
                                        depth + 1, !isMax));
    
                        // Undo the move
                        board[i][j] = '_';
                    }
                }
            }
            return best;
        }
    }
    
    // This will return the best possible
    // move for the player
    public static Move findBestMove(char board[][])
    {
        int bestVal = -1000;
        Move bestMove = new Move();
        bestMove.row = -1;
        bestMove.col = -1;
    
        // Traverse all cells, evaluate minimax function
        // for all empty cells. And return the cell
        // with optimal value.
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                // Check if cell is empty
                if (board[i][j] == '_')
                {
                    // Make the move
                    board[i][j] = player;
    
                    // compute evaluation function for this
                    // move.
                    int moveVal = minimax(board, 0, false);
    
                    // Undo the move
                    board[i][j] = '_';
    
                    // If the value of the current move is
                    // more than the best value, then update
                    // best/
                    if (moveVal > bestVal)
                    {
                        bestMove.row = i;
                        bestMove.col = j;
                        bestVal = moveVal;
                    }
                }
            }
        }
    
        //System.out.printf("The value of the best Move is : %d\n\n", bestVal);
    
        return bestMove;
    }
    
    

    public static void main(String[] args)
    {
        char board[][] = {{ 'x', 'o', 'x' },
                          { 'o', 'o', 'x' },
                          { '_', '_', '_' }};
    
        Move bestMove = findBestMove(board);
    
        System.out.printf("The Optimal Move is :\n");
        System.out.printf("ROW: %d COL: %d\n\n",
                bestMove.row, bestMove.col );
    }
 
}
 
// This code is contributed by PrinciRaj1992
