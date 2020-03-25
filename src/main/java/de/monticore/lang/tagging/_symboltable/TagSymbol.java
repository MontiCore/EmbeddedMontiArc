/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.tagging._symboltable;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Symboltable entry for tag.
 * This is the base class for tags, if you define a new tag
 * this class must be extended
 *
 */
public class TagSymbol extends de.monticore.symboltable.CommonSymbol {

  public static final TagKind KIND = TagKind.INSTANCE;

  protected ArrayList<Object> values = new ArrayList<>();

  public TagSymbol(TagKind kind, Object... values) {
    super("", kind);
    for (Object value : values) {
      this.values.add(value);
    }
  }

  public List<Object> getValues() {
    return this.values;
  }

  /**
   * has no range check, it is should be only used for classes
   * which extends this class, they know how many objects are stored
   */
  protected <T> T getValue(int index) {
    return (T) this.values.get(index);
  }

  /**
   * is only needed for multiple inheritance chains
   */
  protected void addValues(Object... values) {
    for (Object value : values) {
      this.values.add(value);
    }
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append(KIND.getName());
    if (!values.isEmpty()) {
      sb.append("{")
          .append(values.stream().map(v -> v.toString()).collect(Collectors.joining(";")))
          .append("}");
    }
    return sb.toString();
  }

  @Override
  public boolean equals(Object o) {
    if (this == o)
      return true;
    if (!(o instanceof TagSymbol))
      return false;

    TagSymbol tagSymbol = (TagSymbol) o;

    if (this.getKind() != tagSymbol.getKind())
      return false;

    return values.equals(tagSymbol.values);

  }

  @Override
  public int hashCode() {
    return values.hashCode();
  }
}
