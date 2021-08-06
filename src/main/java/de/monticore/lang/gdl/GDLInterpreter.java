package de.monticore.lang.gdl;

import java.io.IOException;
import java.util.Optional;
import java.util.Set;
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
import de.se_rwth.commons.logging.Log;

public class GDLInterpreter {

    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            Log.error("Specify exactly one model file.");
            return;
        }

        Set<String> commands = Set.of(args).stream().filter(arg -> !arg.equals(args[0])).collect(Collectors.toSet());

        boolean chessGui = false;
        boolean cli = true;

        boolean error = false;
        for (String command : commands) {
            if (command.equals("--chess-gui") || command.equals("-cg")) {
                chessGui = true;
            } else if (command.equals("--no-cli") || command.equals("-nc")) {
                cli = false;
            } else {
                Log.error("Unknown command: " + command);
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
        // final IGDLArtifactScope scope = GDLInterpreter.createSymbolTable(ast);

        final Interpreter interpreter = new Interpreter(ast).init();

        if (cli) {
            new Thread(new GDLCLI(interpreter)).start();
        }
        if (chessGui) {
            new ChessGUI(interpreter);
        }
    }

    private static void printHelp() {
        String help = 
            "Usage:\n" +
            "  -cg, --chess-gui" + "\t" + "Start with a Chess GUI\n" +
            "  -nc, --no-cli" + "\t" + "Disable the CLI\n" +
            "";
        System.out.print(help);
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
