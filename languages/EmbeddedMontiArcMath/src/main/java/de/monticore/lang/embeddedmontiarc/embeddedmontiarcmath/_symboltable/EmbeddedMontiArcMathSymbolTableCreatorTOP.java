/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */

package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathVisitor;
import de.monticore.lang.math._ast.ASTMathStatements;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNumberExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueType;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.common2._visitor.Common2Visitor;
import de.se_rwth.commons.logging.Log;

import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;

import java.util.ArrayList;
import java.util.Collection;
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
  public void endVisit(ASTEquation node) {
    EMAMEquationSymbol symbol = new EMAMEquationSymbol();
    symbol.setLeftExpression((MathExpressionSymbol) node.getLeft().getSymbol());
    symbol.setRightExpression((MathExpressionSymbol) node.getRight().getSymbol());

    addToScopeAndLinkWithNode(symbol, node);
  }

  @Override
  public void endVisit(ASTInitialAssignment node) {
    MathMatrixNameExpressionSymbol nameExpressionSymbol = new MathMatrixNameExpressionSymbol(node.getName());
    if(node.isPresentMathMatrixAccessExpression()) {
      nameExpressionSymbol.setMathMatrixAccessOperatorSymbol((MathMatrixAccessOperatorSymbol) node.getMathMatrixAccessExpression().getSymbol());
      ((MathMatrixAccessOperatorSymbol) node.getMathMatrixAccessExpression().getSymbol()).setMathMatrixNameExpressionSymbol(nameExpressionSymbol);
    }
    if (node.isGuess()) {
      EMAMInitialGuessSymbol symbol = new EMAMInitialGuessSymbol(node.getName());
      if(node.isPresentMathMatrixAccessExpression())
        symbol.setMathMatrixAccessOperatorSymbol((MathMatrixAccessOperatorSymbol) node.getMathMatrixAccessExpression().getSymbol());
      symbol.setValue((MathExpressionSymbol) node.getExpression().getSymbol());
      addToScopeAndLinkWithNode(symbol, node);
    } else {
      EMAMInitialValueSymbol symbol = new EMAMInitialValueSymbol(node.getName());
      if(node.isPresentMathMatrixAccessExpression())
        symbol.setMathMatrixAccessOperatorSymbol((MathMatrixAccessOperatorSymbol) node.getMathMatrixAccessExpression().getSymbol());
      symbol.setValue((MathExpressionSymbol) node.getExpression().getSymbol());
      addToScopeAndLinkWithNode(symbol, node);
    }
  }

  @Override
  public void endVisit(ASTSymbolicDeclaration node) {
    for (String name : node.getNameList()) {
      EMAMSymbolicVariableSymbol symbol = new EMAMSymbolicVariableSymbol(name);
      symbol.setType(MathValueType.convert(node.getType()));
      symbol.getType().getProperties().add("Symbolic");
      addToScopeAndLinkWithNode(symbol, node);
    }
  }

  @Override
  public void endVisit(ASTSpecification node) {
    Collection<EMAMSymbolicVariableSymbol> variables = new ArrayList<>();
    Collection<EMAMEquationSymbol> equations = new ArrayList<>();
    Collection<EMAMInitialValueSymbol> initialValues = new ArrayList<>();
    Collection<EMAMInitialGuessSymbol> initialGuesses = new ArrayList<>();
    for (ASTSpecificationStatement astSpecificationStatement : node.getSpecificationStatementList()) {
      if (astSpecificationStatement instanceof ASTSymbolicDeclaration)
        variables.add((EMAMSymbolicVariableSymbol) astSpecificationStatement.getSymbol());
      else if (astSpecificationStatement instanceof ASTInitialAssignment) {
        if (((ASTInitialAssignment) astSpecificationStatement).isGuess())
          initialGuesses.add((EMAMInitialGuessSymbol) astSpecificationStatement.getSymbol());
        else
          initialValues.add((EMAMInitialValueSymbol) astSpecificationStatement.getSymbol());
      } else if (astSpecificationStatement instanceof ASTEquation)
        equations.add((EMAMEquationSymbol) astSpecificationStatement.getSymbol());
    }

    EMAMSpecificationSymbol symbol = new EMAMSpecificationSymbol(variables, equations, initialValues, initialGuesses);
    addToScopeAndLinkWithNode(symbol, node);
  }

  @Override
  public void setRealThis(Common2Visitor realThis) {
    this.realThis = (EmbeddedMontiArcMathVisitor) realThis;
  }
}
