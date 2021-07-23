package de.monticore.lang.gdl.chess;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import de.monticore.lang.gdl.Interpreter2;
import de.monticore.lang.gdl._ast.ASTGameExpression;
import de.monticore.lang.gdl._ast.ASTGameFunction;
import de.monticore.lang.gdl._ast.ASTGameValue;

public class ChessGUI {

    private final Color light = new Color(0x38a2bc);
    private final Color dark = new Color(0x396ABB);
    private final Color selected = new Color(0xE464CC);
    
    private final Color white = new Color(0xFFFFFF);
    private final Color black = new Color(0x000000);

    private final Color error = new Color(0xDC143C);
    
    private final JFrame frame;
    private final Interpreter2 interpreter;

    private String[][] fields;
    private String control;

    private JPanel fieldGrid;
    private JButton[][] fieldButtons;

    private JPanel southPanel;
    private JLabel controlLabel;
    private JButton refreshButton;

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
                final int x = i;
                final int y = j;

                fields[i][j] = "blank";
                fieldButtons[i][j] = new JButton(i + ", " + j) {
                    @Override
                    public void paint(Graphics g) {
                        super.paint(g);
                        g.setColor(Color.BLACK);
                        if (y == 0) {
                            String sX = "" + (char) (x + 97);
                            g.drawString(sX, getWidth()/2 - 2, getHeight() - 20);
                        }
                        if (x == 0) {
                            String sY = "" + (y + 1);
                            g.drawString(sY, 5, getHeight()/2);
                        }
                    }
                };
                fieldButtons[i][j].setBackground(i % 2 == j % 2 ? light : dark);
                fieldButtons[i][j].setFont(fieldButtons[i][j].getFont().deriveFont(18.f).deriveFont(Font.BOLD));
                fieldButtons[i][j].setBorder(BorderFactory.createLineBorder(Color.BLACK));

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

        southPanel = new JPanel(new FlowLayout());
        control = "white";
        controlLabel = new JLabel("Control: " + control);
        controlLabel.setFont(controlLabel.getFont().deriveFont(18.f));
        southPanel.add(controlLabel);

        refreshButton = new JButton("refresh");
        refreshButton.addActionListener((e) -> refresh());
        southPanel.add(refreshButton);

        frame.add(southPanel, BorderLayout.SOUTH);

        updateGameState(interpreter.getGameState());
        frame.setResizable(false);
        frame.pack();
        frame.setVisible(true);
    }

    private void updateGameState(List<ASTGameExpression> gameState) {
        clearAll();

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
                    setFigureOnField(x, y, figure);
                    break;

                case "control":
                    control = ((ASTGameValue) exp.getArguments(0)).getValue();
                    setControlLabel(control);
                    break;
            }
        });
    }

    private void clearAll() {
        for (int x = 0; x < 8; x++) {
            for (int y = 0; y < 8; y++) {
                setFigureOnField(x, y, null);
            }
        }
        setControlLabel("");
    }

    private void setControlLabel(String control) {
        controlLabel.setText("Control: " + control);
    }

    private void setFigureOnField(int x, int y, String figure) {
        fields[x][y] = figure;
        fieldButtons[x][y].setText(textForFigure(figure));
        fieldButtons[x][y].setForeground(colorForFigure(figure));
    }

    private Color colorForFigure(String figure) {
        if (figure == null) {
            return error;
        }

        if (figure.startsWith("white")) {
            return white;
        } else if (figure.startsWith("black")) {
            return black;
        }
        return error;
    }

    private String textForFigure(String figure) {
        String text = "???";

        if (figure != null) {
            switch(figure) {
                case "blank":
                    text = "";
                    break;

                case "white_pawn":
                case "black_pawn":
                    text = "PAWN";
                    break;
                    
                case "white_knight":
                case "black_knight":
                    text = "KNIGHT";
                    break;
                    
                case "white_bishop":
                case "black_bishop":
                    text = "BISHOP";
                    break;
                    
                case "white_rook":
                case "black_rook":
                    text = "ROOK";
                    break;
                    
                case "white_king":
                case "black_king":
                    text = "KING";
                    break;
                    
                case "white_queen":
                case "black_queen":
                    text = "QUEEN";
                    break;
            }
        }

        return text;
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

    private void refresh() {
        updateGameState(interpreter.getGameState());
    }

}
