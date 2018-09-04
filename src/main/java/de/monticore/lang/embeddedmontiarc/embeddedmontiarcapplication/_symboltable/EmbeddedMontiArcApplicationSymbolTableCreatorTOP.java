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

package de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._ast.ASTEMAAplCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._ast.ASTEmbeddedMontiArcApplicationNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._visitor.EmbeddedMontiArcApplicationVisitor;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;

public class EmbeddedMontiArcApplicationSymbolTableCreatorTOP extends de.monticore.symboltable.CommonSymbolTableCreator
         implements EmbeddedMontiArcApplicationVisitor {

  public EmbeddedMontiArcApplicationSymbolTableCreatorTOP(
    final ResolvingConfiguration resolvingConfig, final MutableScope enclosingScope) {
    super(resolvingConfig, enclosingScope);
  }

  public EmbeddedMontiArcApplicationSymbolTableCreatorTOP(final ResolvingConfiguration resolvingConfig, final Deque<MutableScope> scopeStack) {
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
  public Scope createFromAST(ASTEmbeddedMontiArcApplicationNode rootNode) {
    Log.errorIfNull(rootNode, "0xA7004_184 Error by creating of the EmbeddedMontiArcApplicationSymbolTableCreatorTOP symbol table: top ast node is null");
    rootNode.accept(realThis);
    return getFirstCreatedScope();
  }

  private EmbeddedMontiArcApplicationVisitor realThis = this;

  public EmbeddedMontiArcApplicationVisitor getRealThis() {
    return realThis;
  }

  @Override
  public void setRealThis(EmbeddedMontiArcApplicationVisitor realThis) {
    if (this.realThis != realThis) {
      this.realThis = realThis;
    }
  }


  @Override
  public void visit(ASTEMAAplCompilationUnit ast) {
    EMAAplCompilationUnitSymbol eMAMCompilationUnit = create_EMAMCompilationUnit(ast);
    initialize_EMAMCompilationUnit(eMAMCompilationUnit, ast);
    addToScopeAndLinkWithNode(eMAMCompilationUnit, ast);
  }

  protected EMAAplCompilationUnitSymbol create_EMAMCompilationUnit(ASTEMAAplCompilationUnit ast) {
      return new EMAAplCompilationUnitSymbol("");
  }

  protected void initialize_EMAMCompilationUnit(EMAAplCompilationUnitSymbol eMAMCompilationUnit, ASTEMAAplCompilationUnit ast) {

  }


}
