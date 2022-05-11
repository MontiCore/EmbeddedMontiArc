package de.monticore.lang.gdl;

import java.io.IOException;
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
import de.monticore.lang.gdl.visitors.PrologPrinter;
import de.se_rwth.commons.logging.Log;
import de.monticore.lang.gdl._cocos.*;

public class GDLInterpreter {

    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            System.out.println("Specify exactly one model file.");
            return;
        }

        List<String> commands = List.of(args).stream().filter(arg -> !arg.equals(args[0])).collect(Collectors.toList());

        boolean chessGui = false;
        boolean cli = true;
        boolean debugMode = false;
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
            } else if (command.equals("--no-cli") || command.equals("-nc")) {
                cli = false;
            } else if (command.equals("--debug-mode") || command.equals("-dm")) {
                debugMode = true;
            } else {
                System.out.println("Unknown command: " + command);
                error = true;
            }
        }
        if (error) {
            printHelp();
            return;
        }

        // Parse model to ast
        String modelFileName = args[0];
        final ASTGame ast = GDLInterpreter.parse(modelFileName);
        GDLCoCoChecker checker = new GDLCoCoChecker();
        checker.addCoCo(new ASTGameExpressionCoCo());
        checker.checkAll(ast);
        // final IGDLArtifactScope scope = GDLInterpreter.createSymbolTable(ast);

        // PrologPrinter p = new PrologPrinter();
        // ast.accept(p.getTraverser());
        // System.out.println(p.getContent());
        // System.exit(0);

        final Interpreter interpreter = new Interpreter(ast);
        interpreter.setDebugMode(debugMode);
        interpreter.init();

        if (cli) {
            new Thread(new GDLCLI(interpreter)).start();
        }
        if (chessGui) {
            new ChessGUI(interpreter, windowSize);
        }
    }

    private static void printHelp() {
        String help = 
            "Usage:\n" +
            "  -cg, --chess-gui" + "\t" + "Start with a Chess GUI\n" +
            "  -nc, --no-cli" + "\t\t" + "Disable the CLI\n" +
            "  -dm, --debug-mode" + "\t" + "Enable the debug mode\n" +
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
