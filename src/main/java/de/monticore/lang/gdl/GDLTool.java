package de.monticore.lang.gdl;

import de.monticore.lang.gdl.GDLMill;
import de.monticore.lang.gdl._parser.GDLParser;
import de.monticore.lang.gdl._ast.*;
import de.se_rwth.commons.logging.Log;
import org.antlr.v4.runtime.RecognitionException;

import java.io.IOException;
import java.util.Optional;

// import static de.se_rwth.commons.Names.getPathFromPackage;

/**
 * Main class for the Automaton DSL tool.
 *
 */
public class GDLTool {

  /**
   * Use the single argument for specifying the single input automata file.
   *
   * @param args Input arguments.
   */
  public static void main(String[] args) {
    if (args.length != 1) {
      Log.error("Please specify only one single path to the input model.");
      return;
    }
    String model = args[0];

    // parse the model and create the AST representation
    final ASTGame ast = parse(model);
    Log.info(model + " parsed successfully!", GDLTool.class.getName());

    //System.out.println(printGame(ast));

    // setup the symbol table
    // IAutomataArtifactScope modelTopScope = createSymbolTable(ast);

    // can be used for resolving things in the model
    // Optional<StateSymbol> aSymbol = modelTopScope.resolveState("Ping");
    // if (aSymbol.isPresent()) {
    //   Log.info("Resolved state symbol \"Ping\"; FQN = " + aSymbol.get().toString(),
    //       AutomataTool.class.getName());
    // }

    // execute default context conditions
    // runDefaultCoCos(ast);

    // execute a custom set of context conditions
    // AutomataCoCoChecker customCoCos = new AutomataCoCoChecker();
    // customCoCos.addCoCo(new StateNameStartsWithCapitalLetter());
    // customCoCos.checkAll(ast);

    // store artifact scope
    // final AutomataSymbols2Json s2j = new AutomataSymbols2Json();
    // String symFile = "target/symbols/" + getPathFromPackage(modelTopScope.getFullName()) + ".autsym";
    // s2j.store(modelTopScope, symFile);

    // analyze the model with a visitor
    // CountStates cs = new CountStates();
    // AutomataTraverser t = AutomataMill.traverser();
    // t.add4Automata(cs);
    // ast.accept(t);
    // Log.info("The model contains " + cs.getCount() + " states.", AutomataTool.class.getName());

    // execute a pretty printer
    // PrettyPrinter pp = new PrettyPrinter();
    // pp.handle(ast);
    // Log.info("Pretty printing the parsed automata into console:", AutomataTool.class.getName());
    // System.out.println(pp.getResult());
  }

  /*public static String printGame(ASTGame game) {
    StringBuilder builder = new StringBuilder();
    for (ASTGameExpression expr: game.getGameExpressionList()) {
      builder.append(printGameExpression(expr) + "\n");
    }
    return builder.toString();
  }

  public static String printGameExpression(ASTGameExpression expr) {
    StringBuilder builder = new StringBuilder();
    builder.append("(" + expr.getHandle().getName() + (expr.getComponentList().size() == 0 ? "" : " "));
    for (int i = 0; i < expr.getComponentList().size(); i++) {
      ASTGameRelation rel = expr.getComponentList().get(i);
      builder.append(printGameRelation(rel) + (i == expr.getComponentList().size() - 1 ? "" : " "));
    }
    builder.append(")");
    return builder.toString();
  }

  public static String printGameRelation(ASTGameRelation rel) {
    StringBuilder builder = new StringBuilder();
    if (rel instanceof ASTGameString) {
      ASTGameString gString = (ASTGameString) rel;
      builder.append(gString.getName());
    } else if(rel instanceof ASTGameExpression) {
      return printGameExpression((ASTGameExpression) rel);
    } else {
      return "!!!!!!!!!!!!!!!!!! E R R O R !!!!!!!!!!!!!!!!!!";
    }
    return builder.toString();
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
    }
    catch (RecognitionException | IOException e) {
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
  // public static IAutomataArtifactScope createSymbolTable(ASTAutomaton ast) {
  //   IAutomataGlobalScope gs = AutomataMill.globalScope();
  //   gs.clear();

  //   AutomataScopesGenitor genitor = AutomataMill.scopesGenitor();
  //   AutomataTraverser traverser = AutomataMill.traverser();
  //   traverser.setAutomataHandler(genitor);
  //   traverser.add4Automata(genitor);
  //   genitor.putOnStack(gs);

  //   IAutomataArtifactScope scope = genitor.createFromAST(ast);
  //   gs.addSubScope(scope);
  //   return scope;
  // }

  /**
   * Run the default context conditions {@link AtLeastOneInitialAndFinalState},
   * {@link TransitionStatesExist}, and
   * {@link StateNameStartsWithCapitalLetter}.
   *
   * @param ast The input AST.
   */
  // public static void runDefaultCoCos(ASTAutomaton ast) {
  //   new AutomataCoCos().getCheckerForAllCoCos().checkAll(ast);
  // }

}
