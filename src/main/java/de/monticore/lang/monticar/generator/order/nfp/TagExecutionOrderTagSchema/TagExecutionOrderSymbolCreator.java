package de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema;

import de.monticore.lang.tagging._ast.ASTTaggingUnit;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Joiners;

/**
 * Created by ernst on 15.07.2016.
 */
public class TagExecutionOrderSymbolCreator implements TagSymbolCreator {

  public void create(ASTTaggingUnit unit, TaggingResolver gs) {
    if (unit.getQualifiedNames().stream()
        .map(q -> q.toString())
        .filter(n -> n.endsWith("TagExecutionOrderTagSchema"))
        .count() == 0) {
      return; // the tagging model is not conform to the TagExecutionOrderTagSchema tagging schema
    }
    final String packageName = Joiners.DOT.join(unit.getPackage());
    final String rootCmp = // if-else does not work b/cpp of final (required by streams)
        (unit.getTagBody().getTargetModel().isPresent()) ?
            Joiners.DOT.join(packageName,
                unit.getTagBody().getTargetModel().get()
                    .getQualifiedNameString()) :
            packageName;
  }

}
