/* (c) https://github.com/MontiCore/monticore */

package de.monticore.lang.embeddedmontiarc.application._symboltable;

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
