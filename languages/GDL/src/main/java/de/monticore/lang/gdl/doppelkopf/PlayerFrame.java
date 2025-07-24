package de.monticore.lang.gdl.doppelkopf;

import java.awt.Color;
import java.awt.Component;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.border.EmptyBorder;

import de.monticore.lang.gdl.Command;
import de.monticore.lang.gdl.Interpreter;
import de.monticore.lang.gdl.types.GDLTuple;
import de.monticore.lang.gdl.types.GDLType;
import de.monticore.lang.gdl.types.GDLValue;

public class PlayerFrame extends JFrame {

    private final GDLValue player;
    private final Interpreter interpreter;
    private final JCheckBox checkboxRandom;

    public PlayerFrame(String player, Interpreter interpreter) {
        this.player = new GDLValue(player);
        this.interpreter = interpreter;
        setTitle(player);

        JPanel content = new JPanel() {
            public Component add(Component comp) {
                if (comp instanceof JComponent) {
                    JComponent jcomp = (JComponent) comp;
                    jcomp.setAlignmentY(0);

                    jcomp.setBorder(new EmptyBorder(10, 20, 10, 20));
                }
                return super.add(comp);
            }
        };

        checkboxRandom = new JCheckBox("Random", false);

        BoxLayout layout = new BoxLayout(content, BoxLayout.X_AXIS);
        content.setLayout(layout);

        this.setContentPane(content);

        updateGUI(interpreter.getGameStateForRole(this.player));

        interpreter.addObserver(this.player, this::updateGUI);

        setVisible(true);
        setDefaultCloseOperation(EXIT_ON_CLOSE);
    }

    private void updateGUI(Set<GDLType> gameState) {
        this.getContentPane().removeAll();

        if (interpreter.isTerminal()) {
            JPanel panelResult = new JPanel();
            BoxLayout layoutResult = new BoxLayout(panelResult, BoxLayout.Y_AXIS);
            panelResult.setLayout(layoutResult);

            JLabel gameOverLabel = new JLabel();
            gameOverLabel.setText("Game Over");
            gameOverLabel.setFont(gameOverLabel.getFont().deriveFont(16.f));
            panelResult.add(gameOverLabel);

            JLabel resultLabel = new JLabel();
            resultLabel.setText(interpreter.getGoals().toString());
            panelResult.add(resultLabel);

            this.getContentPane().add(panelResult);
            pack();
            return;
        }

        // Spielerhand
        JPanel panelHand = new JPanel();
        BoxLayout layoutHand = new BoxLayout(panelHand, BoxLayout.Y_AXIS);
        panelHand.setLayout(layoutHand);

        JLabel titleHand = new JLabel("Hand ");
        titleHand.setFont(titleHand.getFont().deriveFont(16.f));
        panelHand.add(titleHand);

        this.getContentPane().add(panelHand);

        // Stiche
        JPanel panelStiche = new JPanel();
        BoxLayout layoutStiche = new BoxLayout(panelStiche, BoxLayout.Y_AXIS);
        panelStiche.setLayout(layoutStiche);

        JLabel titleStiche = new JLabel("Stiche ");
        titleStiche.setFont(titleHand.getFont());
        panelStiche.add(titleStiche);

        this.getContentPane().add(panelStiche);

        // Aktueller Stich
        JPanel panelAktStich = new JPanel();
        BoxLayout layoutAktStich = new BoxLayout(panelAktStich, BoxLayout.Y_AXIS);
        panelAktStich.setLayout(layoutAktStich);

        JLabel titleAktStich = new JLabel("Akt. Stich ");
        titleAktStich.setFont(titleHand.getFont());
        panelAktStich.add(titleAktStich);

        this.getContentPane().add(panelAktStich);

        // Teams
        JPanel panelTeam = new JPanel();
        BoxLayout layoutTeam = new BoxLayout(panelTeam, BoxLayout.Y_AXIS);
        panelTeam.setLayout(layoutTeam);

        JLabel titleTeam = new JLabel("Team ");
        titleTeam.setFont(titleHand.getFont());
        panelTeam.add(titleTeam);

        this.getContentPane().add(panelTeam);

        // Ablauf
        JPanel panelAblauf = new JPanel();
        BoxLayout layoutAblauf = new BoxLayout(panelAblauf, BoxLayout.Y_AXIS);
        panelAblauf.setLayout(layoutAblauf);

        JLabel titleAblauf = new JLabel("Ablauf ");
        titleAblauf.setFont(titleHand.getFont());
        panelAblauf.add(titleAblauf);

        this.getContentPane().add(panelAblauf);

        // Sonstige
        JPanel panelSonstige = new JPanel();
        BoxLayout layoutSonstige = new BoxLayout(panelSonstige, BoxLayout.Y_AXIS);
        panelSonstige.setLayout(layoutSonstige);

        JLabel titleSonstige = new JLabel("Sonstige ");
        titleSonstige.setFont(titleHand.getFont());
        panelSonstige.add(titleSonstige);

        this.getContentPane().add(panelSonstige);

        for (GDLType stateGDL : gameState) {
            if (!(stateGDL instanceof GDLTuple)) {
                continue;
            }
            GDLTuple stateTuple = (GDLTuple) stateGDL;
            List<String> state = stateTuple.stream()
                    .map(e -> e.toString())
                    .collect(Collectors.toList()) ;

            switch (state.get(0)) {
                case "hand":
                    panelHand.add(new JLabel(state.get(2)));
                    break;

                case "team":
                    panelTeam.add(new JLabel(state.get(2) + ": " + state.get(1)));
                    break;

                case "ablauf":
                case "zug":
                case "spielart":
                case "hidden_spielart":
                case "spiel":
                case "klaerung":
                    panelAblauf.add(new JLabel(state.stream().reduce("", (x, y) -> x + ", " + y).substring(2)));
                    break;

                case "stich":
                    if (state.get(1).equals(getAblaufStichMajor(gameState))) {
                        panelAktStich.add(new JLabel(state.stream().skip(1).reduce("", (x, y) -> x + ", " + y).substring(2)));
                    } else {
                        panelStiche.add(new JLabel(state.stream().skip(1).reduce("", (x, y) -> x + ", " + y).substring(2)));
                    }
                    break;

                default:
                    panelSonstige.add(new JLabel(state.stream().reduce("", (x, y) -> x + ", " + y).substring(2)));
                    break;
            }

        }


        // Legal:
        JPanel panelLegal = new JPanel();
        BoxLayout layoutLegal = new BoxLayout(panelLegal, BoxLayout.Y_AXIS);
        panelLegal.setLayout(layoutLegal);

        JLabel titleLegal = new JLabel("Legal ");
        titleLegal.setFont(titleHand.getFont());
        panelLegal.add(titleLegal);

        panelLegal.add(checkboxRandom);

        this.getContentPane().add(panelLegal);

        List<Command> commands = new ArrayList<>(interpreter.getAllLegalMovesForRole(player));

        for (final Command c : commands) {
            JButton commandButton = new JButton(c.toString());
            if (c.toString().contains("spielen")) {
                commandButton.setBackground(new Color(0x8dfc8d));
            } else if (c.toString().contains("noop")) {
                commandButton.setBackground(new Color(0x8dfbfc));
            } else {
                commandButton.setBackground(new Color(0xedfc8d));
            }

            commandButton.addActionListener((e) -> {
                interpreter.interpret(c);
                Arrays.stream(panelLegal.getComponents()).skip(2).forEach(bt -> ((JButton) bt).setEnabled(false));
            });

            panelLegal.add(commandButton);
        }


        titleHand.setText(titleHand.getText() + "(" + (panelHand.getComponentCount() - 1) + ") :");
        titleStiche.setText(titleStiche.getText() + "(" + (panelStiche.getComponentCount() - 1) + ") :");
        titleAktStich.setText(titleAktStich.getText() + "(" + (panelAktStich.getComponentCount() - 1) + ") :");
        titleTeam.setText(titleTeam.getText() + "(" + (panelTeam.getComponentCount() - 1) + ") :");
        titleAblauf.setText(titleAblauf.getText() + "(" + (panelAblauf.getComponentCount() - 1) + ") :");
        titleSonstige.setText(titleSonstige.getText() + "(" + (panelSonstige.getComponentCount() - 1) + ") :");
        titleLegal.setText(titleLegal.getText() + "(" + (panelLegal.getComponentCount() - 2) + ") :");


        this.pack();
        repaint();

        if (this.checkboxRandom.isSelected() && !commands.isEmpty()) {
            Arrays.stream(panelLegal.getComponents()).skip(2).forEach(bt -> ((JButton) bt).setEnabled(false));
            Command c = commands.get((int) (Math.random() * commands.size()));
            interpreter.interpret(c);
        }
    }

    private String getAblaufStichMajor(Set<GDLType> gameState) {
        Optional<GDLType> maybeMajor = gameState.stream()
                .filter(t -> (t instanceof GDLTuple))
                .map(t -> (GDLTuple) t)
                .filter(t -> t.get(0).equals(new GDLValue("ablauf")))
                .filter(t -> t.get(1).equals(new GDLValue("stich")))
                .map(t -> t.get(2))
                .findFirst();

        if (maybeMajor.isPresent()) {
            return maybeMajor.get().toString();
        } else {
            return null;
        }
    }
    
}
