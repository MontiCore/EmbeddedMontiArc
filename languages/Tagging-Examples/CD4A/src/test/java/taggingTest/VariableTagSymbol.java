/* (c) https://github.com/MontiCore/monticore */
/* generated by template templates.de.monticore.lang.tagschema.ValuedTagType*/


package taggingTest;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;


/**
 * Created by ValuedTagType.ftl
 */
public class VariableTagSymbol extends TagSymbol {
  public static final VariableTagKind KIND = VariableTagKind.INSTANCE;

  public VariableTagSymbol(String value) {
    super(KIND, value);
  }

  protected VariableTagSymbol(VariableTagKind kind, String value) {
    super(kind, value);
  }

  public String getValue() {
     return getValue(0);
  }

  @Override
  public String toString() {
    return String.format("VariableTag = \"%s\"",
      getValue().toString());
  }

  public static class VariableTagKind extends TagKind {
    public static final VariableTagKind INSTANCE = new VariableTagKind();

    protected VariableTagKind() {
    }
  }
}
