package de.monticore.lang.monticar.utilities.utils;

public enum JarClassifier {

  DATASET("dataset"),
  EMADL("emadl"),
  PRETRAINED("pretrained"),
  EMPTY("");

  public final String value;

  private JarClassifier(String value) {
    this.value = value;
  }

}
