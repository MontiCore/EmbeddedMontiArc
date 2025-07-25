/* (c) https://github.com/MontiCore/monticore */
/* generated by template templates.de.monticore.lang.tagschema.ValuedTagType*/


package taggingTest;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;


/**
 * Created by ValuedTagType.ftl
 */
public class ClassTagSymbol extends TagSymbol {
  public static final ClassTagKind KIND = ClassTagKind.INSTANCE;

  public ClassTagSymbol(String value) {
    super(KIND, value);
  }

  protected ClassTagSymbol(ClassTagKind kind, String value) {
    super(kind, value);
  }

  public String getValue() {
     return getValue(0);
  }

  @Override
  public String toString() {
    return String.format("ClassTag = \"%s\"",
      getValue().toString());
  }

  public static class ClassTagKind extends TagKind {
    public static final ClassTagKind INSTANCE = new ClassTagKind();

    protected ClassTagKind() {
    }
  }
}
