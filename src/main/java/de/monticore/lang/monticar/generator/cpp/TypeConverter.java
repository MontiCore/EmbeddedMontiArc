/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import static de.monticore.lang.monticar.generator.cpp.TypeConverter.CppTypes.BOOL;
import static de.monticore.lang.monticar.generator.cpp.TypeConverter.CppTypes.DOUBLE;
import static de.monticore.lang.monticar.generator.cpp.TypeConverter.EmbeddedMontiArcTypes.ASSIGNMENT_TYPE;
import static de.monticore.lang.monticar.generator.cpp.TypeConverter.EmbeddedMontiArcTypes.B;
import static de.monticore.lang.monticar.generator.cpp.TypeConverter.EmbeddedMontiArcTypes.C;
import static de.monticore.lang.monticar.generator.cpp.TypeConverter.EmbeddedMontiArcTypes.COMMON_MATRIX_TYPE;
import static de.monticore.lang.monticar.generator.cpp.TypeConverter.EmbeddedMontiArcTypes.Q;
import static de.monticore.lang.monticar.generator.cpp.TypeConverter.EmbeddedMontiArcTypes.SI_UNIT_RANGES_TYPE;
import static de.monticore.lang.monticar.generator.cpp.TypeConverter.EmbeddedMontiArcTypes.UNIT_NUMBER_RESOLUTION;
import static de.monticore.lang.monticar.generator.cpp.TypeConverter.EmbeddedMontiArcTypes.Z;

import java.util.EnumSet;

/**
 * This enum contains constants for EmbeddedMontiArc data types and C++ data types and provides
 * functionality to convert between them. The conversions are not really optimal as for example
 * natural numbers in EmbeddedMontiArc are mapped to double in C++.
 * <p>
 * The data type constants are not necessarily complete.
 */
public enum TypeConverter {
  RANGE(SI_UNIT_RANGES_TYPE, DOUBLE),
  BOOLEAN(B, BOOL),
  INTEGERS(Z, DOUBLE),
  RATIONAL_NUMBER(Q, DOUBLE),
  COMPLEX_NUMBERS(C, DOUBLE),
  UNIT_NUMBER(UNIT_NUMBER_RESOLUTION, DOUBLE),
  MATRIX(COMMON_MATRIX_TYPE, CppTypes.MATRIX),
  ASSIGNMENT(ASSIGNMENT_TYPE, CppTypes.MATRIX);

  private final EmbeddedMontiArcTypes embeddedMontiArcType;
  private final CppTypes cppTypes;

  TypeConverter(EmbeddedMontiArcTypes embeddedMontiArcType, CppTypes cppTypes) {
    this.embeddedMontiArcType = embeddedMontiArcType;
    this.cppTypes = cppTypes;
  }

  /**
   * Returns a C++ data type that is equivalent or similar to the provided EmbeddedMontiArc
   * data type. If the provided data type name is unknown, a string saying {@code "null"} is
   * returned.
   *
   * @param montiArcType EmbeddedMontiArc data type name
   * @return converted C++ data type name or {@code "null"}
   * @see #getConversion(EmbeddedMontiArcTypes)
   */
  public static String getConversion(String montiArcType) {
    return getConversion(EmbeddedMontiArcTypes.get(montiArcType)).getType();
  }

  /**
   * Returns a {@link CppTypes} data type that is equivalent or similar to the provided
   * {@link EmbeddedMontiArcTypes} data type. If a conversion is not possible or not yet
   * implemented, {@code CppTypes.NULL} is returned.
   *
   * @param montiArcType EmbeddedMontiArc data type
   * @return converted C++ data type or {@code CppTypes.NULL}
   */
  public static CppTypes getConversion(EmbeddedMontiArcTypes montiArcType) {
    return EnumSet.allOf(TypeConverter.class).stream()
        .filter(c -> c.embeddedMontiArcType == montiArcType)
        .map(c -> c.cppTypes)
        .findFirst().orElse(CppTypes.NULL);
  }

  public enum EmbeddedMontiArcTypes {
    SI_UNIT_RANGES_TYPE("SIUnitRangesType"),
    B("B"),
    Z("Z"),
    Q("Q"),
    C("C"),
    UNIT_NUMBER_RESOLUTION("UnitNumberResolution"),
    COMMON_MATRIX_TYPE("CommonMatrixType"),
    ASSIGNMENT_TYPE("AssignmentType"),
    NULL("null");

    private final String type;

    EmbeddedMontiArcTypes(String type) {
      this.type = type;
    }

    /**
     * Returns the enum constant corresponding to the provided EmbeddedMontiArc data type name. If
     * the data type name is unknown, {@code EmbeddedMontiArcTypes.NULL} is returned.
     *
     * @param type EmbeddedMontiArc data type name
     * @return corresponding {@code EmbeddedMontiArcTypes} constant or {@code EmbeddedMontiArcTypes.NULL}
     */
    public static EmbeddedMontiArcTypes get(String type) {
      return EnumSet.allOf(EmbeddedMontiArcTypes.class).stream()
          .filter(t -> t.getType().equals(type)).findFirst().orElse(NULL);
    }

    public String getType() {
      return type;
    }
  }

  public enum CppTypes {
    DOUBLE("double"),
    BOOL("bool"),
    MATRIX("mat"),
    NULL("null");

    private final String type;

    CppTypes(String type) {
      this.type = type;
    }

    /**
     * Returns the enum constant corresponding to the provided C++ data type name. If the data type
     * name is unknown, {@code CppTypes.NULL} is returned.
     *
     * @param type C++ data type name
     * @return corresponding {@code CppTypes} constant or {@code CppTypes.NULL}
     */
    public static CppTypes get(String type) {
      return EnumSet.allOf(CppTypes.class).stream()
          .filter(t -> t.getType().equals(type)).findFirst().orElse(NULL);
    }

    public String getType() {
      return type;
    }
  }
}
