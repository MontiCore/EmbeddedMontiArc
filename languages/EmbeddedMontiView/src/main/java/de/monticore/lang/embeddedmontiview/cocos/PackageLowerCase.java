/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTEMVCompilationUnit;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTEMVCompilationUnitCoCo;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.EmbeddedMontiArcModelNameCalculator;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;

/**
 * Ensures, that packages of components are lower-case. This is required for inner components, see
 * {@link EmbeddedMontiArcModelNameCalculator}.
 *
 */
public class PackageLowerCase
    implements EmbeddedMontiViewASTEMVCompilationUnitCoCo {

  /**
   * @see EmbeddedMontiViewASTEMVCompilationUnitCoCo#check(ASTEMVCompilationUnit)
   */
  @Override
  public void check(ASTEMVCompilationUnit node) {
    String pack = Names.getQualifiedName(node.getPackage());
    if (pack.toUpperCase().equals(pack)) {
      Log.error("0xAC003 The package must be lower case", node.get_SourcePositionStart());
    }
  }

}
