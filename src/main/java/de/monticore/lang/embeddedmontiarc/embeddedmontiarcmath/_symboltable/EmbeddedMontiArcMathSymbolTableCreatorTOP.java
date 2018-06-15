/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */

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


  @Override
  public void visit(de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit ast) {
    EMAMCompilationUnitSymbol eMAMCompilationUnit = create_EMAMCompilationUnit(ast);
    initialize_EMAMCompilationUnit(eMAMCompilationUnit, ast);
    addToScopeAndLinkWithNode(eMAMCompilationUnit, ast);
  }

  protected EMAMCompilationUnitSymbol create_EMAMCompilationUnit(de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit ast) {
      return new EMAMCompilationUnitSymbol("");
  }

  protected void initialize_EMAMCompilationUnit(EMAMCompilationUnitSymbol eMAMCompilationUnit, de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit ast) {

  }

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
