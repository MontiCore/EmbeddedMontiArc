/* (c) https://github.com/MontiCore/monticore */
/*package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTComponent;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTMontiArcInvariant;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;*/

/**
 * Checks that names of invariants within a component are unique.
 *
 */
/*public class UniqueConstraint implements EmbeddedMontiViewASTComponentCoCo {

  /**
   * @see EmbeddedMontiViewASTComponentCoCo#check(ASTComponent)
   */
 /* @Override
  public void check(ASTComponent node) {
    final List<String> usedNames = new ArrayList<String>();

    for (ASTMontiArcInvariant inv : node.getBody().getElements().stream()
        .filter(a -> a instanceof ASTMontiArcInvariant).map(a -> (ASTMontiArcInvariant) a)
        .collect(Collectors.toList())) {

      if (usedNames.contains(inv.getName())) {
        Log.error(String.format("0xAC001 The name of constraint '%s' is ambiguos!", inv.getName()),
            inv.get_SourcePositionStart());
      }

      usedNames.add(inv.getName());
    }
  }

}*/
