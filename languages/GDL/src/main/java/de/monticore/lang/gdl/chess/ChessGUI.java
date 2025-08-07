package de.monticore.lang.gdl.chess;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.GridLayout;
import java.awt.Image;
import java.io.IOException;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;

import de.monticore.lang.gdl.Command;
import de.monticore.lang.gdl.Interpreter;
import de.monticore.lang.gdl.types.GDLTuple;
import de.monticore.lang.gdl.types.GDLType;

public class ChessGUI {

    private ImageIcon black_king, black_queen, black_rook, black_knight, black_bishop, black_pawn;
    private ImageIcon white_king, white_queen, white_rook, white_knight, white_bishop, white_pawn;

    private final Color light = new Color(0xc7ddf2);
    private final Color dark = new Color(0x407fb7);
    private final Color selected = new Color(0xF6F669);
    
    private final Color white = new Color(0xFFFFFF);
    private final Color black = new Color(0x000000);

    private final Color error = new Color(0xDC143C);
    
    private final JFrame frame;
    private final Interpreter interpreter;

    private String[][] fields;
    private String control;

    private JPanel fieldGrid;
    private JButton[][] fieldButtons;

    private JPanel southPanel;
    private JLabel controlLabel;
    private JButton refreshButton;

    private boolean isSelected;
    private int selectX, selectY;

    public ChessGUI(Interpreter interpreter, int windowSize) {
        initIcons(windowSize);

        this.frame = new JFrame("Chess");
        frame.setLayout(new BorderLayout());
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);

        this.interpreter = interpreter;

        fieldGrid = new JPanel();
        GridLayout layout = new GridLayout(8, 8);
        fieldGrid.setLayout(layout);
        fieldGrid.setPreferredSize(new Dimension(windowSize, windowSize));

        fields = new String[8][8];
        fieldButtons = new JButton[8][8];
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                final int x = i;
                final int y = j;

                fields[i][j] = null;
                fieldButtons[i][j] = new JButton() {
                    @Override
                    public void paint(Graphics g) {
                        super.paint(g);
                        g.setColor(Color.BLACK);
                        if (y == 0) {
                            String sX = "" + (char) (x + 97);
                            g.drawString(sX, getWidth() - 14, getHeight() - 5);
                        }
                        if (x == 0) {
                            String sY = "" + (y + 1);
                            g.drawString(sY, 5, 20);
                        }
                    }
                };
                fieldButtons[i][j].setBackground(i % 2 == j % 2 ? dark : light);
                fieldButtons[i][j].setFont(fieldButtons[i][j].getFont().deriveFont(18.f).deriveFont(Font.BOLD));
                // fieldButtons[i][j].setBorder(BorderFactory.createLineBorder(Color.BLACK));
                fieldButtons[i][j].setBorder(null);

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

        updateGameState(interpreter.getVisibleGameState());
        frame.setResizable(false);
        frame.pack();
        frame.setVisible(true);
    }

    private void initIcons(int windowSize) {
        final int size = windowSize/8;
        try {
            black_king = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/black_king.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));
            black_queen = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/black_queen.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));
            black_rook = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/black_rook.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));
            black_knight = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/black_knight.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));
            black_bishop = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/black_bishop.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));
            black_pawn = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/black_pawn.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));

            white_king = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/white_king.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));
            white_queen = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/white_queen.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));
            white_rook = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/white_rook.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));
            white_knight = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/white_knight.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));
            white_bishop = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/white_bishop.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));
            white_pawn = new ImageIcon(ImageIO.read(ChessGUI.class.getResource("figures/white_pawn.png")).getScaledInstance(size, size, Image.SCALE_SMOOTH));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void updateGameState(Set<GDLType> gameState) {
        clearAll();

        gameState.stream()
                .filter(gdlType -> (gdlType instanceof GDLTuple))
                .map(gdlType -> (GDLTuple) gdlType)
                .forEach(exp -> {
                    String identifier = exp.get(0).toString();

                    switch (identifier) {
                        case "field":
                            String sX = exp.get(1).toString();
                            String sY = exp.get(2).toString();
                            String figure = exp.get(3).toString();
                            int x = sX.charAt(0) - 97;
                            int y = Integer.valueOf(sY) - 1;
                            setFigureOnField(x, y, figure);
                            break;

                        case "control":
                            control = exp.get(1).toString();
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
        // fieldButtons[x][y].setText(textForFigure(figure));
        fieldButtons[x][y].setIcon(iconForFigure(figure));
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

    // private static String textForFigure(String figure) {
    //     String text = "???";

    //     if (figure != null) {
    //         switch(figure) {
    //             case "blank":
    //                 text = "";
    //                 break;

    //             case "white_pawn":
    //             case "black_pawn":
    //                 text = "PAWN";
    //                 break;
                    
    //             case "white_knight":
    //             case "black_knight":
    //                 text = "KNIGHT";
    //                 break;
                    
    //             case "white_bishop":
    //             case "black_bishop":
    //                 text = "BISHOP";
    //                 break;
                    
    //             case "white_rook":
    //             case "black_rook":
    //                 text = "ROOK";
    //                 break;
                    
    //             case "white_king":
    //             case "black_king":
    //                 text = "KING";
    //                 break;
                    
    //             case "white_queen":
    //             case "black_queen":
    //                 text = "QUEEN";
    //                 break;
    //         }
    //     }

    //     return text;
    // }

    private ImageIcon iconForFigure(String figure) {
        if (figure == null) {
            return null;
        }

        ImageIcon icon = null;
        switch(figure) {
            case "blank":
                break;

            case "white_pawn":
                icon = white_pawn;
                break;
            case "black_pawn":
                icon = black_pawn;
                break;
                
            case "white_knight":
                icon = white_knight;
                break;
            case "black_knight":
                icon = black_knight;
                break;
                
            case "white_bishop":
                icon = white_bishop;
                break;
            case "black_bishop":
                icon = black_bishop;
                break;
                
            case "white_rook":
                icon = white_rook;
                break;
            case "black_rook":
                icon = black_rook;
                break;
                
            case "white_king":
                icon = white_king;
                break;
            case "black_king":
                icon = black_king;
                break;
                
            case "white_queen":
                icon = white_queen;
                break;
            case "black_queen":
                icon = black_queen;
                break;
        }

        return icon;
    }

    private void select(int x, int y) {
        if (!isSelected) {
            isSelected = true;
            fieldButtons[x][y].setBackground(selected);
            selectX = x;
            selectY = y;
        } else if (isSelected && x == selectX && y == selectY) {
            isSelected = false;
            fieldButtons[x][y].setBackground(x % 2 == y % 2 ? dark : light);
        } else {
            move(control, selectX, selectY, x, y);
            isSelected = false;
            fieldButtons[selectX][selectY].setBackground(selectX % 2 == selectY % 2 ? dark : light);
        }
    }

    private void move(String player, int x, int y, int tX, int tY) {
        String figure = fields[x][y];
        String sX = "" + (char) (x + 97);
        String sY = "" + (y + 1);
        String sTX = "" + (char) (tX + 97);
        String sTY = "" + (tY + 1);
        
        String move = String.format("%s (move %s %s %s %s %s)", player, figure, sX, sY, sTX, sTY);

        boolean legal = interpreter.interpret(Command.createFromLine(move));
        if (legal) {
            Set<GDLType> nextState = interpreter.getVisibleGameState();
            updateGameState(nextState);

            if (interpreter.isTerminal()) {
                Map<GDLType, GDLType> goals = interpreter.getGoals();
                StringBuilder sb = new StringBuilder();
                for (Entry<GDLType, GDLType> goal : goals.entrySet()) {
                    sb.append(String.format("\tPlayer %s achieved %s points.\n", goal.getKey(), goal.getValue()));
                }
                JOptionPane.showMessageDialog(frame, sb.toString(), "Game Over!", JOptionPane.INFORMATION_MESSAGE);
                System.exit(0);
            }
        } else {
            String message = "Move was not legal! Move:\n\t" + move;
            JOptionPane.showMessageDialog(frame, message, "Illegal Move", JOptionPane.WARNING_MESSAGE);
        }
    }

    private void refresh() {
        updateGameState(interpreter.getVisibleGameState());
    }

}
