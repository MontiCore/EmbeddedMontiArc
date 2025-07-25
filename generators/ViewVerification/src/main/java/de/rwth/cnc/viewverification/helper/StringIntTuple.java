/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.helper;

public class StringIntTuple {

  private int integer;
  private String string;

  public StringIntTuple(String str) {
    string = str;
    integer = 0;
  }

  public StringIntTuple(String str, int integer) {
    string = str;
    this.integer = integer;
  }

  public String getString() {
    return string;
  }

  public int getInteger() {
    return integer;
  }

  public void incrementInteger() {
    integer++;
  }

  public void decrementInteger() {
    integer--;
  }
}
