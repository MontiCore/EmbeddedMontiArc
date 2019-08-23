/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.tagging._ast;

import java.util.Optional;

public class ASTTagElement extends ASTTagElementTOP {

  protected ASTTagElement(String name, Optional<String> tagValue) {
    super(name, tagValue);
  }

  protected ASTTagElement () {
    super();
  }

  public void setTagValue(String tagValue) {
    if (tagValue != null) {
      if (tagValue.startsWith("=")) {
        tagValue = tagValue.substring(1);
      }
      if (tagValue.endsWith(";")) {
        tagValue = tagValue.substring(0, tagValue.length() - 1);
      }
      tagValue = tagValue.trim();
    }
    super.setTagValue(tagValue);
  }
}


