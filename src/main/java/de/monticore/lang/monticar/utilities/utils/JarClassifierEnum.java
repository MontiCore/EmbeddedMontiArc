package de.monticore.lang.monticar.utilities.utils;

public enum JarClassifierEnum {

  DATASET("dataset"),
  EMADL("emadl"),
  PRETRAINED("pretrained"),
  EMPTY("");

  public final String value;

  JarClassifierEnum(String value) {
    this.value = value;
  }

}
