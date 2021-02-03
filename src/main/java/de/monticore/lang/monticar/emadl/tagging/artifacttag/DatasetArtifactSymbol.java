package de.monticore.lang.monticar.emadl.tagging.artifacttag;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

public class DatasetArtifactSymbol extends TagSymbol {

  public static final DatasetArtifactKind KIND = DatasetArtifactKind.INSTANCE;

  public DatasetArtifactSymbol() {
    super(KIND, ".");
  }

  public DatasetArtifactSymbol(String artifact, String jar, String id) {
    this(KIND, artifact, jar, id);
  }

  public DatasetArtifactSymbol(DatasetArtifactKind kind, String artifact, String jar, String id) {
    super(kind, artifact, jar, id);
  }

  public String getJar() {
    return getValue(1);
  }

  public String getId() {
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
