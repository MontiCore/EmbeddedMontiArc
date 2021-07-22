package de.monticore.lang.gdl.chess;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import de.monticore.lang.gdl.Interpreter2;
import de.monticore.lang.gdl._ast.ASTGameExpression;
import de.monticore.lang.gdl._ast.ASTGameFunction;
import de.monticore.lang.gdl._ast.ASTGameValue;

public class ChessGUI {

    private final Color light = new Color(0xEEEED2);
    private final Color dark = new Color(0x769656);
    private final Color selected = new Color(0xBACA2B);
    
    private final JFrame frame;
    private final Interpreter2 interpreter;

    private String[][] fields;
    private String control;

    private JPanel fieldGrid;
    private JButton[][] fieldButtons;
    private JLabel controlLabel;

    private boolean isSelected;
    private int selectX, selectY;

    public ChessGUI(Interpreter2 interpreter) {
        this.frame = new JFrame("Chess");
        frame.setLayout(new BorderLayout());
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        this.interpreter = interpreter;

        fieldGrid = new JPanel();
        GridLayout layout = new GridLayout(8, 8);
        fieldGrid.setLayout(layout);
        fieldGrid.setPreferredSize(new Dimension(1000, 1000));

        fields = new String[8][8];
        fieldButtons = new JButton[8][8];
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                fields[i][j] = "blank";
                fieldButtons[i][j] = new JButton(i + ", " + j);
                fieldButtons[i][j].setBackground(i % 2 == j % 2 ? light : dark);
                fieldButtons[i][j].setFont(fieldButtons[i][j].getFont().deriveFont(16.f));

                final int x = i;
                final int y = j;

                fieldButtons[i][j].addActionListener(e -> {
                    select(x, y);
                });
            }
        }

        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                fieldGrid.add(fieldButtons[j][7-i]);
            }
        }
        frame.add(fieldGrid);

        control = "white";
        controlLabel = new JLabel("Control: " + control);
        controlLabel.setFont(controlLabel.getFont().deriveFont(18.f));
        frame.add(controlLabel, BorderLayout.SOUTH);

        updateGameState(interpreter.getGameState());
        frame.setResizable(false);
        frame.pack();
        frame.setVisible(true);
    }

    private void updateGameState(List<ASTGameExpression> gameState) {
        for (int x = 0; x < 8; x++) {
            for (int y = 0; y < 8; y++) {
                fields[x][y] = "";
                fieldButtons[x][y].setText(fields[x][y]);
            }
        }
        controlLabel.setText("Control: ");

        System.out.println("New Game State (" + gameState.size() + "):");

        gameState.forEach(exp -> {
            String identifier = ((ASTGameFunction) exp.getType()).getFunction();

            System.out.println("\t" + exp);

            switch (identifier) {
                case "field":
                    String sX = ((ASTGameValue) exp.getArguments(0)).getValue();
                    String sY = ((ASTGameValue) exp.getArguments(1)).getValue();
                    String figure = ((ASTGameValue) exp.getArguments(2)).getValue();
                    int x = sX.charAt(0) - 97;
                    int y = Integer.valueOf(sY) - 1;
                    fields[x][y] = figure;
                    fieldButtons[x][y].setText(fields[x][y]);
                    break;

                case "control":
                    control = ((ASTGameValue) exp.getArguments(0)).getValue();
                    controlLabel.setText("Control: " + control);
                    break;
            }
        });
    }

    private void select(int x, int y) {
        if (!isSelected) {
            isSelected = true;
            fieldButtons[x][y].setBackground(selected);
            selectX = x;
            selectY = y;
        } else if (isSelected && x == selectX && y == selectY) {
            isSelected = false;
            fieldButtons[x][y].setBackground(x % 2 == y % 2 ? light : dark);
        } else {
            move(control, selectX, selectY, x, y);
            isSelected = false;
            fieldButtons[selectX][selectY].setBackground(selectX % 2 == selectY % 2 ? light : dark);
        }
    }

    private void move(String player, int x, int y, int tX, int tY) {
        String figure = fields[x][y];
        String sX = "" + (char) (x + 97);
        String sY = "" + (y + 1);
        String sTX = "" + (char) (tX + 97);
        String sTY = "" + (tY + 1);
        
        String move = String.format("%s (move %s %s %s %s %s)", player, figure, sX, sY, sTX, sTY);

        List<ASTGameExpression> nextState = interpreter.interpret(move);
        if (nextState != null) {
            updateGameState(nextState);
        } else {
            System.out.println("Move was not legal! Move:");
            System.out.println("\t" + move);
        }
    }

}
