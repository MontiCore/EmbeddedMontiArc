package de.monticore.lang.monticar.generator.js;

import static de.monticore.lang.monticar.generator.js.JsGenerator.getDimension;
import static de.monticore.lang.monticar.generator.js.JsGenerator.getLowerBound;
import static de.monticore.lang.monticar.generator.js.JsGenerator.getUnit;
import static de.monticore.lang.monticar.generator.js.JsGenerator.getUpperBound;
import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.freemarker.TemplateProcessor;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.ResolverFactory;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collection;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

@SuppressWarnings("ConstantConditions")
class JsGeneratorTest {

  private static final Path RESOLVING_BASE_DIR = Paths.get("src/test/resources/jsfilegeneration");
  private static final String IN = "in";
  private static final String OUT = "out";

  private static final String RANGE_MODEL = "models.multiplePorts";
  private static final String BOOL = "Bool";
  private static final String UNBOUNDED = "Unbounded";
  private static final String BOUNDED = "Bounded";
  private static final String STEP = "Step";
  private static final String UNIT = "Unit";
  private static final String UPPER_BOUND = "UpperBound";
  private static final String LOWER_BOUND = "LowerBound";
  private static final String STEP_UNIT = "StepUnit";

  private static final String DIMENSION_MODEL = "models.dimensions";
  private static final String SCALAR = "Scalar";
  private static final String ARRAY = "Array[1]";
  private static final String ROW_VECTOR = "RowVector";
  private static final String COLUMN_VECTOR = "ColumnVector";
  private static final String MATRIX = "Matrix";
  private static final String MATRIX_ARRAY = "MatrixArray[2]";

  private ExpandedComponentInstanceSymbol model;

  private static String portName(String direction, String name) {
    return direction + name;
  }

  @BeforeEach
  void setUp() {
    ResolverFactory resolverFactory = new ResolverFactory(RESOLVING_BASE_DIR);
    model = resolverFactory.get().getExpandedComponentInstanceSymbol(RANGE_MODEL);
  }

  @Nested
  class Generate {

    ExpandedComponentInstanceSymbol rangeModel;

    @BeforeEach
    void setUp() {
      ResolverFactory resolverFactory = new ResolverFactory(RESOLVING_BASE_DIR);
      Resolver resolver = resolverFactory.get();
      rangeModel = resolver.getExpandedComponentInstanceSymbol(RANGE_MODEL);
    }

    @Test
    void shouldGenerateCpp() throws IOException, TemplateException {
      TemplateProcessor templateProcessor = mock(TemplateProcessor.class);
      JsGenerator jsGenerator = new JsGenerator(templateProcessor);

      jsGenerator.generate(rangeModel);

      verify(templateProcessor).process(any());
    }
  }

  @Nested
  class GetUnit {

    @Nested
    class ShouldReturnNull {

      @Nested
      class GivenIncomingPort {

        @Test
        void whenBooleanPort() {
          PortSymbol port = model.getIncomingPort(portName(IN, BOOL)).get();

          assertThat(getUnit(port)).isNull();
        }

        @Test
        void whenNoRangeDefined() {
          PortSymbol port = model.getIncomingPort(portName(IN, UNBOUNDED)).get();

          assertThat(getUnit(port)).isNull();
        }

        @Test
        void whenNoUnitDefinedWithoutStepSize() {
          PortSymbol port = model.getIncomingPort(portName(IN, BOUNDED)).get();

          assertThat(getUnit(port)).isNull();
        }

        @Test
        void whenNoUnitDefinedWithStepSize() {
          PortSymbol port = model.getIncomingPort(portName(IN, STEP)).get();

          assertThat(getUnit(port)).isNull();
        }
      }

      @Nested
      class GivenOutgoingPort {

        @Test
        void whenBooleanPort() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, BOOL)).get();

          assertThat(getUnit(port)).isNull();
        }

        @Test
        void whenNoRangeDefined() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, UNBOUNDED)).get();

          assertThat(getUnit(port)).isNull();
        }

        @Test
        void whenNoUnitDefinedWithoutStepSize() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, BOUNDED)).get();

          assertThat(getUnit(port)).isNull();
        }

        @Test
        void whenNoUnitDefinedWithStepSize() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, STEP)).get();

          assertThat(getUnit(port)).isNull();
        }
      }
    }

    @Nested
    class ShouldReturnUnitString {

      @Nested
      class GivenIncomingPort {

        @Test
        void whenUnbounded() {
          PortSymbol port = model.getIncomingPort(portName(IN, UNIT)).get();

          assertThat(getUnit(port)).isEqualTo("mN");
        }

        @Test
        void whenUpperBound() {
          PortSymbol port = model.getIncomingPort(portName(IN, UPPER_BOUND)).get();

          assertThat(getUnit(port)).isEqualTo("m");
        }

        @Test
        void whenLowerBound() {
          PortSymbol port = model.getIncomingPort(portName(IN, LOWER_BOUND)).get();

          assertThat(getUnit(port)).isEqualTo("m/h");
        }

        @Test
        void whenBounden() {
          PortSymbol port = model.getIncomingPort(portName(IN, STEP_UNIT)).get();

          assertThat(getUnit(port)).isEqualTo("MW");
        }
      }

      @Nested
      class GivenOutgoingPort {

        @Test
        void whenUnbounded() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, UNIT)).get();

          assertThat(getUnit(port)).isEqualTo("mN");
        }

        @Test
        void whenUpperBound() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, UPPER_BOUND)).get();

          assertThat(getUnit(port)).isEqualTo("m");
        }

        @Test
        void whenLowerBound() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, LOWER_BOUND)).get();

          assertThat(getUnit(port)).isEqualTo("m/h");
        }

        @Test
        void whenBounden() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, STEP_UNIT)).get();

          assertThat(getUnit(port)).isEqualTo("MW");
        }
      }
    }
  }

  @Nested
  class GetLowerBound {

    @Nested
    class ShouldReturnNull {

      @Nested
      class GivenIncomingPort {

        @Test
        void whenBooleanPort() {
          PortSymbol port = model.getIncomingPort(portName(IN, BOOL)).get();

          assertThat(getLowerBound(port)).isNull();
        }

        @Test
        void whenNoRangeDefined() {
          PortSymbol port = model.getIncomingPort(portName(IN, UNBOUNDED)).get();

          assertThat(getLowerBound(port)).isNull();
        }

        @Test
        void whenBoundIsInfinite() {
          PortSymbol port = model.getIncomingPort(portName(IN, STEP)).get();

          assertThat(getLowerBound(port)).isNull();
        }
      }

      @Nested
      class GivenOutgoingPort {

        @Test
        void whenBooleanPort() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, BOOL)).get();

          assertThat(getLowerBound(port)).isNull();
        }

        @Test
        void whenNoRangeDefined() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, UNBOUNDED)).get();

          assertThat(getLowerBound(port)).isNull();
        }

        @Test
        void whenBoundIsInfinite() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, STEP)).get();

          assertThat(getLowerBound(port)).isNull();
        }
      }
    }

    @Nested
    class ShouldReturnValueOnly {

      @Nested
      class GivenIncomingPort {

        @Test
        void whenNoUnitDefined() {
          PortSymbol port = model.getIncomingPort(portName(IN, BOUNDED)).get();

          assertThat(getLowerBound(port)).isEqualTo("-5/2");
        }
      }

      @Nested
      class GivenOutgoingPort {

        @Test
        void whenNoUnitDefined() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, BOUNDED)).get();

          assertThat(getLowerBound(port)).isEqualTo("-5/2");
        }
      }
    }

    @Nested
    class ShouldReturnValueAndUnit {

      @Nested
      class GivenIncomingPort {

        @Test
        void whenUnitDefinedWithoutStep() {
          PortSymbol port = model.getIncomingPort(portName(IN, LOWER_BOUND)).get();

          assertThat(getLowerBound(port)).isEqualTo("1/1 m/h");
        }

        @Test
        void whenUnitDefinedWithStep() {
          PortSymbol port = model.getIncomingPort(portName(IN, STEP_UNIT)).get();

          assertThat(getLowerBound(port)).isEqualTo("-10/1 MW");
        }
      }

      @Nested
      class GivenOutgoingPort {

        @Test
        void whenUnitDefinedWithoutStep() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, LOWER_BOUND)).get();

          assertThat(getLowerBound(port)).isEqualTo("1/1 m/h");
        }

        @Test
        void whenUnitDefinedWithStep() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, STEP_UNIT)).get();

          assertThat(getLowerBound(port)).isEqualTo("-10/1 MW");
        }
      }
    }
  }

  @Nested
  class GetUpperBound {

    @Nested
    class ShouldReturnNull {

      @Nested
      class GivenIncomingPort {

        @Test
        void whenBooleanPort() {
          PortSymbol port = model.getIncomingPort(portName(IN, BOOL)).get();

          assertThat(getUpperBound(port)).isNull();
        }

        @Test
        void whenNoRangeDefined() {
          PortSymbol port = model.getIncomingPort(portName(IN, UNBOUNDED)).get();

          assertThat(getUpperBound(port)).isNull();
        }

        @Test
        void whenBoundIsInfinite() {
          PortSymbol port = model.getIncomingPort(portName(IN, STEP)).get();

          assertThat(getUpperBound(port)).isNull();
        }
      }

      @Nested
      class GivenOutgoingPort {

        @Test
        void whenBooleanPort() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, BOOL)).get();

          assertThat(getUpperBound(port)).isNull();
        }

        @Test
        void whenNoRangeDefined() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, UNBOUNDED)).get();

          assertThat(getUpperBound(port)).isNull();
        }

        @Test
        void whenBoundIsInfinite() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, STEP)).get();

          assertThat(getUpperBound(port)).isNull();
        }
      }
    }

    @Nested
    class ShouldReturnValueOnly {

      @Nested
      class GivenIncomingPort {

        @Test
        void whenNoUnitDefined() {
          PortSymbol port = model.getIncomingPort(portName(IN, BOUNDED)).get();

          assertThat(getUpperBound(port)).isEqualTo("5/1");
        }
      }

      @Nested
      class GivenOutgoingPort {

        @Test
        void whenNoUnitDefined() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, BOUNDED)).get();

          assertThat(getUpperBound(port)).isEqualTo("5/1");
        }
      }
    }

    @Nested
    class ShouldReturnValueAndUnit {

      @Nested
      class GivenIncomingPort {

        @Test
        void whenUnitDefinedWithoutStep() {
          PortSymbol port = model.getIncomingPort(portName(IN, UPPER_BOUND)).get();

          assertThat(getUpperBound(port)).isEqualTo("10/1 km");
        }

        @Test
        void whenUnitDefinedWithStep() {
          PortSymbol port = model.getIncomingPort(portName(IN, STEP_UNIT)).get();

          assertThat(getUpperBound(port)).isEqualTo("20/1 kW");
        }
      }

      @Nested
      class GivenOutgoingPort {

        @Test
        void whenUnitDefinedWithoutStep() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, UPPER_BOUND)).get();

          assertThat(getUpperBound(port)).isEqualTo("10/1 km");
        }

        @Test
        void whenUnitDefinedWithStep() {
          PortSymbol port = model.getOutgoingPort(portName(OUT, STEP_UNIT)).get();

          assertThat(getUpperBound(port)).isEqualTo("20/1 kW");
        }
      }
    }
  }

  @Nested
  class GetDimension {

    private ExpandedComponentInstanceSymbol dimensionModel;

    @BeforeEach
    void setUp() {
      ResolverFactory resolverFactory = new ResolverFactory(RESOLVING_BASE_DIR);
      Resolver resolver = resolverFactory.get();
      dimensionModel = resolver.getExpandedComponentInstanceSymbol(DIMENSION_MODEL);
    }

    @Nested
    class ShouldReturnNull {

      @Nested
      class GivenIncomingPort {

        @Test
        void whenScalar() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getIncomingPort(portName(IN, SCALAR)).get();

          assertThat(getDimension(ports, port)).isNull();
        }
      }

      @Nested
      class GivenOutgoingPort {

        @Test
        void whenScalar() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getOutgoingPort(portName(OUT, SCALAR)).get();

          assertThat(getDimension(ports, port)).isNull();
        }
      }
    }

    @Nested
    class ShouldReturnDimensionArray {

      @Nested
      class GivenIncomingPort {

        @Test
        void whenArray() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getIncomingPort(portName(IN, ARRAY)).get();

          assertThat(getDimension(ports, port)).containsExactly(3);
        }

        @Test
        void whenRowVector() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getIncomingPort(portName(IN, ROW_VECTOR)).get();

          assertThat(getDimension(ports, port)).containsExactly(1, 3);
        }

        @Test
        void whenColumnVector() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getIncomingPort(portName(IN, COLUMN_VECTOR)).get();

          assertThat(getDimension(ports, port)).containsExactly(4, 1);
        }

        @Test
        void whenMatrix() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getIncomingPort(portName(IN, MATRIX)).get();

          assertThat(getDimension(ports, port)).containsExactly(2, 3);
        }

        @Test
        void whenMatrixArray() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getIncomingPort(portName(IN, MATRIX_ARRAY)).get();

          assertThat(getDimension(ports, port)).containsExactly(2, 3, 4);
        }
      }

      @Nested
      class GivenOutgoingPort {

        @Test
        void whenArray() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getOutgoingPort(portName(OUT, ARRAY)).get();

          assertThat(getDimension(ports, port)).containsExactly(3);
        }

        @Test
        void whenRowVector() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getOutgoingPort(portName(OUT, ROW_VECTOR)).get();

          assertThat(getDimension(ports, port)).containsExactly(1, 3);
        }

        @Test
        void whenColumnVector() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getOutgoingPort(portName(OUT, COLUMN_VECTOR)).get();

          assertThat(getDimension(ports, port)).containsExactly(4, 1);
        }

        @Test
        void whenMatrix() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getOutgoingPort(portName(OUT, MATRIX)).get();

          assertThat(getDimension(ports, port)).containsExactly(2, 3);
        }

        @Test
        void whenMatrixArray() {
          Collection<PortSymbol> ports = dimensionModel.getPorts();
          PortSymbol port = dimensionModel.getOutgoingPort(portName(OUT, MATRIX_ARRAY)).get();

          assertThat(getDimension(ports, port)).containsExactly(2, 3, 4);
        }
      }
    }
  }
}