/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.tagging._symboltable;

import de.monticore.lang.tagging._ast.ASTTaggingUnit;
import de.monticore.symboltable.Scope;

/**
 * Created by Michael von Wenckstern on 31.05.2016.
 *
 * @author Michael von Wenckstern
 */
public interface TagSymbolCreator {
  void create(ASTTaggingUnit unit, TaggingResolver scope);
}

