/* (c) https://github.com/MontiCore/monticore */
/* generated by template templates.de.monticore.lang.tagschema.SimpleTagType*/


package de.monticore.lang.montiarc.tag.drawing.TraceabilityTagSchema;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

/**
 * Created by SimpleTagType.ftl
 */
public class TraceableSymbol extends TagSymbol {
  public static final TraceableKind KIND = TraceableKind.INSTANCE;

  /**
   * is marker symbol so it has no value by itself
   */
  public TraceableSymbol() {
    super(KIND);
  }

  protected TraceableSymbol(TraceableKind kind) {
    super(kind);
  }

  @Override
  public String toString() {
    return "Traceable";
  }

  public static class TraceableKind extends TagKind {
    public static final TraceableKind INSTANCE = new TraceableKind();

    protected TraceableKind() {
    }
  }
}
