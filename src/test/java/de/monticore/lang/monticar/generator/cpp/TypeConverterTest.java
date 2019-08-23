/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import static org.assertj.core.api.Assertions.assertThat;
import static org.quicktheories.QuickTheory.qt;
import static org.quicktheories.generators.SourceDSL.strings;

import de.monticore.lang.monticar.generator.cpp.TypeConverter.CppTypes;
import de.monticore.lang.monticar.generator.cpp.TypeConverter.EmbeddedMontiArcTypes;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class TypeConverterTest {

  private static final String EMBEDDED_MONTIARC_TYPE_BOOLEAN_STRING = "B";
  private static final String CPP_TYPE_BOOLEAN_STRING = "bool";
  private static final EmbeddedMontiArcTypes EMBEDDED_MONTIARC_TYPE_BOOLEAN = EmbeddedMontiArcTypes.B;
  private static final CppTypes CPP_TYPE_BOOLEAN = CppTypes.BOOL;

  private static final String NOT_EMBEDDED_MONTIARC_TYPE_STRING = "notype";
  private static final String EMBEDDED_MONTIARC_TYPE_MATRIX_STRING = "CommonMatrixType";
  private static final EmbeddedMontiArcTypes EMBEDDED_MONTIARC_TYPE_MATRIX = EmbeddedMontiArcTypes.COMMON_MATRIX_TYPE;

  private static final String NOT_CPP_TYPE_STRING = "notype";
  private static final String CPP_TYPE_DOUBLE_STRING = "double";
  private static final CppTypes CPP_TYPE_DOUBLE = CppTypes.DOUBLE;

  @Nested
  class GetConversion {

    @Nested
    class ShouldReturnValueNull {

      @Test
      void whenStringIsNotEmbeddedMontiArcType() {
        assertThat(TypeConverter.getConversion(NOT_EMBEDDED_MONTIARC_TYPE_STRING))
            .isEqualTo(CppTypes.NULL.getType());
      }
    }

    @Nested
    class ShouldReturnCorrectConversion {

      @Test
      void whenGivenMontiArcType() {
        assertThat(TypeConverter.getConversion(EMBEDDED_MONTIARC_TYPE_BOOLEAN))
            .isEqualTo(CPP_TYPE_BOOLEAN);
      }

      @Test
      void whenGivenMontiArcTypeString() {
        assertThat(TypeConverter.getConversion(EMBEDDED_MONTIARC_TYPE_BOOLEAN_STRING))
            .isEqualTo(CPP_TYPE_BOOLEAN_STRING);
      }
    }
  }

  @Nested
  class EmbeddedMontiArcTypesTest {

    @Nested
    class Get {

      @Nested
      class ShouldNotReturnNull {

        @Test
        void whenGivenAnyString() {
          qt().forAll(strings().allPossible().ofLengthBetween(0, 10))
              .check(s -> EmbeddedMontiArcTypes.get(s) != null);
        }
      }

      @Nested
      class ShouldReturnValueNull {

        @Test
        void whenStringIsNotEmbeddedMontiArcType() {
          assertThat(EmbeddedMontiArcTypes.get(NOT_EMBEDDED_MONTIARC_TYPE_STRING))
              .isEqualTo(EmbeddedMontiArcTypes.NULL);
        }
      }

      @Nested
      class ShouldReturnCorrectValue {

        @Test
        void whenGivenExistingType() {
          assertThat(EmbeddedMontiArcTypes.get(EMBEDDED_MONTIARC_TYPE_MATRIX_STRING))
              .isEqualTo(EMBEDDED_MONTIARC_TYPE_MATRIX);
        }
      }
    }
  }

  @Nested
  class CppTypesTest {

    @Nested
    class Get {

      @Nested
      class ShouldNotReturnNull {

        @Test
        void whenGivenAnyString() {
          qt().forAll(strings().allPossible().ofLengthBetween(0, 10))
              .check(s -> CppTypes.get(s) != null);
        }
      }

      @Nested
      class ShouldReturnValueNull {

        @Test
        void whenStringIsNotCppType() {
          assertThat(CppTypes.get(NOT_CPP_TYPE_STRING))
              .isEqualTo(CppTypes.NULL);
        }
      }

      @Nested
      class ShouldReturnCorrectValue {

        @Test
        void whenGivenExistingType() {
          assertThat(CppTypes.get(CPP_TYPE_DOUBLE_STRING))
              .isEqualTo(CPP_TYPE_DOUBLE);
        }
      }
    }
  }

}
