/* (c) https://github.com/MontiCore/monticore */

package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTBehaviorEmbedding;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathVisitor;
import de.monticore.lang.math._ast.ASTMathStatements;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.common2._visitor.Common2Visitor;
import de.se_rwth.commons.logging.Log;

import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import java.util.Deque;

public class EmbeddedMontiArcMathSymbolTableCreatorTOP extends de.monticore.symboltable.CommonSymbolTableCreator
         implements EmbeddedMontiArcMathVisitor {

  public EmbeddedMontiArcMathSymbolTableCreatorTOP(
    final ResolvingConfiguration resolvingConfig, final MutableScope enclosingScope) {
    super(resolvingConfig, enclosingScope);
  }

  public EmbeddedMontiArcMathSymbolTableCreatorTOP(final ResolvingConfiguration resolvingConfig, final Deque<MutableScope> scopeStack) {
    super(resolvingConfig, scopeStack);
  }

  private void initSuperSTC() {
  }

  /**
  * Creates the symbol table starting from the <code>rootNode</code> and
  * returns the first scope that was created.
  *
  * @param rootNode the root node
  * @return the first scope that was created
  */
  public Scope createFromAST(de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEmbeddedMontiArcMathNode rootNode) {
    Log.errorIfNull(rootNode, "0xA7004_184 Error by creating of the EmbeddedMontiArcMathSymbolTableCreatorTOP symbol table: top ast node is null");
    rootNode.accept(realThis);
    return getFirstCreatedScope();
  }

  private EmbeddedMontiArcMathVisitor realThis = this;

  public EmbeddedMontiArcMathVisitor getRealThis() {
    return realThis;
  }

  @Override
  public void setRealThis(EmbeddedMontiArcMathVisitor realThis) {
    if (this.realThis != realThis) {
      this.realThis = realThis;
    }
  }


//  @Override
//  public void visit(de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit ast) {
//    EMAMCompilationUnitSymbol eMAMCompilationUnit = create_EMAMCompilationUnit(ast);
//    initialize_EMAMCompilationUnit(eMAMCompilationUnit, ast);
//    addToScopeAndLinkWithNode(eMAMCompilationUnit, ast);
//  }

//  protected EMAMCompilationUnitSymbol create_EMAMCompilationUnit(de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit ast) {
//      return new EMAMCompilationUnitSymbol("");
//  }
//
//  protected void initialize_EMAMCompilationUnit(EMAMCompilationUnitSymbol eMAMCompilationUnit, de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit ast) {
//
//  }

  @Override
  public void visit(ASTBehaviorEmbedding node) {
    // create a math statements symbol here
    ASTMathStatements astMathStatements = new ASTMathStatements();
    astMathStatements.statements = node.getStatementList();
    addToScopeAndLinkWithNode(new MathStatementsSymbol("MathStatements", astMathStatements), node);
  }

  @Override
  public void setRealThis(Common2Visitor realThis) {

  }
}
