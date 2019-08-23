/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.tagschema._ast;

import java.util.Optional;

/**
 * Created by MichaelvonWenckstern on 28.06.2016.
 */
public class ASTComplexTagType extends ASTComplexTagTypeTOP {
  public ASTComplexTagType() {
    super();
  }

  public ASTComplexTagType(String name, Optional<ASTScope> scope, String complexTag) {
    super(name, scope, complexTag);
  }

  public void setComplexTag(String complexTag) {
    if (complexTag != null) {
      if (complexTag.startsWith("is")) {
        complexTag = complexTag.substring(2);
      }
      if (complexTag.endsWith(";")) {
        complexTag = complexTag.substring(0, complexTag.length() - 1);
      }
    }
    super.setComplexTag(complexTag);
  }
}
