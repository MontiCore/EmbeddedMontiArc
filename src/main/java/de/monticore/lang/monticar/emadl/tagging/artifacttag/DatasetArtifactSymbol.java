/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.tagging.artifacttag;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

public class DatasetArtifactSymbol extends TagSymbol {

  public static final DatasetArtifactKind KIND = DatasetArtifactKind.INSTANCE;

  public DatasetArtifactSymbol() {
    super(KIND, ".");
  }

  public DatasetArtifactSymbol(String artifact, String jar, String type) {
    this(KIND, artifact, jar, type);
  }

  public DatasetArtifactSymbol(DatasetArtifactKind kind, String artifact, String jar, String type) {
    super(kind, artifact, jar, type);
  }

  public String getArtifact() {
    return getValue(0);
  }

  public String getJar() {
    return getValue(1);
  }

  public String getType() {
    return getValue(2);
  }

  @Override
  public String toString() {
    return super.toString();
  }

  public static class DatasetArtifactKind extends TagKind {
    public static final DatasetArtifactKind INSTANCE = new DatasetArtifactKind();

    protected DatasetArtifactKind() {}
  }

}
