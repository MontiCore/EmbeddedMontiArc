package de.monticore.lang.gdl;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Optional;
import java.util.List;
import java.util.stream.Collectors;

import org.antlr.v4.runtime.RecognitionException;

import de.monticore.lang.gdl._ast.ASTGame;
import de.monticore.lang.gdl._parser.GDLParser;
import de.monticore.lang.gdl._symboltable.GDLScopesGenitor;
import de.monticore.lang.gdl._symboltable.IGDLArtifactScope;
import de.monticore.lang.gdl._symboltable.IGDLGlobalScope;
import de.monticore.lang.gdl._visitor.GDLTraverser;
import de.monticore.lang.gdl.chess.ChessGUI;
import de.monticore.lang.gdl.cli.GDLCLI;
import de.monticore.lang.gdl.doppelkopf.DoppelkopfGUI;
import de.se_rwth.commons.logging.Log;

public class GDLInterpreter {

    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            System.out.println("Specify exactly one model file.");
            printHelp();
            return;
        }

        List<String> commands = List.of(args).stream().filter(arg -> !arg.equals(args[0])).collect(Collectors.toList());

        boolean chessGui = false;
        boolean doppelkopfGui = false;
        boolean cli = true;
        boolean debugMode = false;
        boolean manualRandom = false;
        boolean showTimes = false;
        boolean withTypes = false;
        boolean output = false;
        String outputPath = "out.pl";
        int windowSize = 950;

        boolean error = false;
        for (int i = 0; i < commands.size(); i++) {
            String command = commands.get(i);
            if (command.equals("--chess-gui") || command.equals("-cg")) {
                chessGui = true;
                if (i + 1 < commands.size() && commands.get(i + 1).matches("[0-9]+")) {
                    windowSize = Integer.parseInt(commands.get(i + 1));
                    i++;
                }
            } else if(command.equals("--doppelkopf-gui") || command.equals("-dg")) {
                doppelkopfGui = true;
            } else if (command.equals("--no-cli") || command.equals("-nc")) {
                cli = false;
            } else if (command.equals("--debug-mode") || command.equals("-dm")) {
                debugMode = true;
            } else if (command.equals("--manual-random") || command.equals("-mr")) {
                manualRandom = true;
            } else if (command.equals("--show-times") || command.equals("-st")) {
                showTimes = true;
            }  else if (command.equals("--with-types") || command.equals("-wt")) {
                withTypes = true;
            } else if (command.equals("--out") || command.equals("-o")) {
                output = true;
                if (i + 1 < commands.size()) {
                    outputPath = commands.get(i + 1);
                    i++;
                }
            } else {
                System.out.println("Unknown command: " + command);
                error = true;
            }
        }
        if (error) {
            printHelp();
            return;
        }

        String gdlFilePath = args[0];
        InterpreterOptions options = new InterpreterOptions()
                .debugMode(debugMode)
                .manualRandom(manualRandom)
                .showTimes(showTimes)
                .withTypes(withTypes);
        
        final Interpreter interpreter = Interpreter.fromGDLFile(gdlFilePath, options);

        if (output) {
            writeOutput(outputPath, interpreter.getPrologProgram());
        }

        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            try {
                interpreter.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }));

        if (cli) {
            new Thread(new GDLCLI(interpreter)).start();
        }
        if (chessGui) {
            new ChessGUI(interpreter, windowSize);
        }
        if (doppelkopfGui) {
            new DoppelkopfGUI(interpreter);
        }
    }

    private static void writeOutput(String outputPath,  String prologProgram) {
        try {
            Files.writeString(new File(outputPath).toPath(), prologProgram, StandardCharsets.UTF_8);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void printHelp() {
        String help = 
            "Usage:  gdl-cli [path-to-gdl-model] {options}\n" +
            "\n" +
            "Options:\n"+
            "  -o,  --out" + "\t\t" + "Write Prolog output file\n" +
            "  -cg, --chess-gui" + "\t" + "Start with a Chess GUI\n" +
            "  -dg, --doppelkopf-gui" + "\t" + "Start with a Doppelkopf GUI\n" +
            "  -nc, --no-cli" + "\t\t" + "Disable the CLI\n" +
            "  -dm, --debug-mode" + "\t" + "Enable the debug mode\n" +
            "  -mr, --manual-random" + "\t" + "Enable manual control over the random role\n" +
            "  -st, --show-times" + "\t" + "Profile the runtime of some functions\n" +
            "  -wt, --with-types" + "\t" + "Create features for strongly typed models\n" +
            "";
        System.out.println(help);
    }

    /**
     * Parse the model contained in the specified file.
     *
     * @param model - file to parse
     * @return AST of the model.
     */
    public static ASTGame parse(String model) {
        try {
            GDLParser parser = GDLMill.parser();
            Optional<ASTGame> optGame = parser.parse(model);

            if (!parser.hasErrors() && optGame.isPresent()) {
                return optGame.get();
            }
            Log.error("Model could not be parsed.");
        } catch (RecognitionException | IOException e) {
            Log.error("Failed to parse " + model, e);
        }
        return null;
    }

    /**
     * Create the symbol table from the parsed AST.
     *
     * @param ast Input AST.
     * @return The symbol table created from the AST.
     */
    public static IGDLArtifactScope createSymbolTable(ASTGame ast) {
        IGDLGlobalScope gs = GDLMill.globalScope();
        gs.clear();

        GDLScopesGenitor genitor = GDLMill.scopesGenitor();
        GDLTraverser traverser = GDLMill.traverser();
        traverser.setGDLHandler(genitor);
        traverser.add4GDL(genitor);
        genitor.putOnStack(gs);

        IGDLArtifactScope scope = genitor.createFromAST(ast);
        gs.addSubScope(scope);
        return scope;
    }

}
