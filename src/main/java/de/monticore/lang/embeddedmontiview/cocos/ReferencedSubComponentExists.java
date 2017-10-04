/*
 * Copyright (c) 2017, MontiCore. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.helper.ArcTypePrinter;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTSubComponentCoCo;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ComponentSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

/**
 * Implements R3 and R4 from AHs PhD thesis
 * 
 * @author Crispin Kirchner
 */
public class ReferencedSubComponentExists implements EmbeddedMontiViewASTSubComponentCoCo {
  
  /**
   * @see de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentBodyCoCo#check(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponentBody)
   */
  @Override
  public void check(ASTSubComponent node) {
    String referenceName = ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(node.getType());
    
    Scope scope = node.getEnclosingScope().get();
    
    Optional<ComponentSymbol> componentSymbol = scope
        .<ComponentSymbol> resolve(referenceName, ComponentSymbol.KIND);
    
    if (!componentSymbol.isPresent()) {
      Log.error(String.format("0x069B7 Type \"%s\" could not be resolved", referenceName),
          node.get_SourcePositionStart());
    }
  }
  
}
