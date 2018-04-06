package de.monticore.lang.monticar.generator.js;

import static de.monticore.lang.monticar.generator.GeneratorUtil.filterMultipleArrayPorts;
import static de.monticore.lang.monticar.generator.GeneratorUtil.getGetterMethodName;
import static de.monticore.lang.monticar.generator.GeneratorUtil.getSetterMethodName;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonDimensionElement;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.freemarker.TemplateProcessor;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTType;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import org.jscience.mathematics.number.Rational;

public class JsGenerator {

  private static final String PARAMETER_NAME = "param";
  private final TemplateProcessor templateProcessor;

  public JsGenerator(TemplateProcessor templateProcessor) {
    this.templateProcessor = templateProcessor;
  }



  static String getUnit(PortSymbol port) {
    Optional<ASTRange> rangeOpt = getRange(port);
    String unit = rangeOpt.map(astRange -> astRange.getUnit().toString()).orElse("");
    return unit.isEmpty() ? null : unit;
  }

  static String getLowerBound(PortSymbol port) {
    Optional<ASTRange> rangeOpt = getRange(port);
    if (rangeOpt.isPresent()) {
      ASTRange range = rangeOpt.get();
      String bound = join(" ", getLowerBoundValue(range), getLowerBoundUnit(range));
      return bound.isEmpty() ? null : bound;
    }
    return null;
  }

  static String getUpperBound(PortSymbol port) {
    Optional<ASTRange> rangeOpt = getRange(port);
    if (rangeOpt.isPresent()) {
      ASTRange range = rangeOpt.get();
      String bound = join(" ", getUpperBoundValue(range), getUpperBoundUnit(range));
      return bound.isEmpty() ? null : bound;
    }
    return null;
  }

  private static String getLowerBoundUnit(ASTRange range) {
    if (range.hasStartUnit()) {
      return range.getStartUnit().toString();
    }
    return "";
  }

  private static String getUpperBoundUnit(ASTRange range) {
    if (range.hasEndUnit()) {
      return range.getEndUnit().toString();
    }
    return "";
  }

  private static String getLowerBoundValue(ASTRange range) {
    if (!range.hasNoLowerLimit()) {
      Rational startValue = range.getStartValue();
      return startValue.toString();
    }
    return "";
  }

  private static String getUpperBoundValue(ASTRange range) {
    if (!range.hasNoUpperLimit()) {
      Rational startValue = range.getEndValue();
      return startValue.toString();
    }
    return "";
  }

  static int[] getDimension(Collection<PortSymbol> ports, PortSymbol port) {
    int arrayDimension = port.isPartOfPortArray() ?
        getArrayDimension(ports, port.getNameWithoutArrayBracketPart()) : 0;
    int[] matrixDimension = getMatrixDimension(port);
    return combineDimensions(arrayDimension, matrixDimension);
  }

  private static int[] combineDimensions(int arrayDimension, int[] matrixDimension) {
    if (arrayDimension > 0 && matrixDimension.length > 0) {
      int[] dimension = new int[matrixDimension.length + 1];
      dimension[0] = arrayDimension;
      System.arraycopy(matrixDimension, 0, dimension, 1, matrixDimension.length);
      return dimension;
    } else if (matrixDimension.length > 0) {
      return matrixDimension;
    } else if (arrayDimension > 0) {
      return new int[]{arrayDimension};
    } else {
      return null;
    }
  }

  private static int getArrayDimension(Collection<PortSymbol> ports, String arrayName) {
    int dimension = 0;
    for (PortSymbol port : ports) {
      if (port.getNameWithoutArrayBracketPart().equals(arrayName)) {
        dimension++;
      }
    }
    return dimension;
  }

  private static int[] getMatrixDimension(PortSymbol port) {
    MCTypeReference<? extends MCTypeSymbol> typeReference = port.getTypeReference();
    if (typeReference instanceof MCASTTypeSymbolReference) {
      ASTType type = ((MCASTTypeSymbolReference) typeReference).getAstType();
      if (type instanceof ASTCommonMatrixType) {
        ASTCommonMatrixType matrixType = (ASTCommonMatrixType) type;
        List<ASTCommonDimensionElement> dimensionElements = matrixType.getCommonDimension()
            .getCommonDimensionElements();
        int[] dimensions = new int[dimensionElements.size()];
        for (int i = 0; i < dimensionElements.size(); i++) {
          final int index = i;
          dimensionElements.get(i).getUnitNumber().ifPresent(unitNumber -> {
            unitNumber.getNumber().ifPresent(dim -> dimensions[index] = dim.intValue());
          });
        }
        return dimensions;
      }
    }
    return new int[0];
  }

  private static Optional<ASTRange> getRange(PortSymbol port) {
    MCASTTypeSymbolReference typeReference = (MCASTTypeSymbolReference) port.getTypeReference();
    ASTType astType = typeReference.getAstType();
    if (astType instanceof ASTCommonMatrixType) {
      ASTCommonMatrixType type = (ASTCommonMatrixType) astType;
      return type.getElementType().getRange();
    } else if (astType instanceof ASTElementType) {
      ASTElementType type = (ASTElementType) astType;
      return type.getRange();
    } else {
      throw new RuntimeException("Unexpected ASTType: " + astType);
    }
  }

  private static String join(String delimiter, String... elements) {
    return Arrays.stream(elements).filter(s -> !s.isEmpty()).collect(Collectors.joining(delimiter));
  }

  public void generate(ExpandedComponentInstanceSymbol symbol)
      throws IOException, TemplateException {
    Collection<PortSymbol> outports = filterMultipleArrayPorts(symbol.getOutgoingPorts());
    Collection<PortSymbol> inports = filterMultipleArrayPorts(symbol.getIncomingPorts());
    List<Getter> getters = produceGetters(outports);
    List<Setter> setters = produceSetters(inports, symbol.getIncomingPorts());

    Map<String, Object> dataModel = new HashMap<>();
    dataModel.put("getters", getters);
    dataModel.put("setters", setters);

    templateProcessor.process(dataModel);
  }

  private List<Getter> produceGetters(Collection<PortSymbol> outgoingPorts) {
    List<Getter> getters = new ArrayList<>();
    for (PortSymbol port : outgoingPorts) {
      Getter getter = new Getter();
      String methodName = getGetterMethodName(port);
      getter.setMethodName(methodName);
      getter.setDelegateMethodName(methodName);
      getter.setUnit(getUnit(port));
      getters.add(getter);
    }
    return getters;
  }

  private List<Setter> produceSetters(Collection<PortSymbol> incomingPorts,
      Collection<PortSymbol> rawIncomingPorts) {
    List<Setter> setters = new ArrayList<>();
    for (PortSymbol port : incomingPorts) {
      Setter setter = new Setter();
      String methodName = getSetterMethodName(port);
      setter.setMethodName(methodName);
      setter.setParameterName('_' + port.getNameWithoutArrayBracketPart());
      setter.setDelegateMethodName(methodName);
      setter.setDimension(getDimension(rawIncomingPorts, port));
      setter.setUnit(getUnit(port));
      setter.setLowerBound(getLowerBound(port));
      setter.setUpperBound(getUpperBound(port));
      setters.add(setter);
    }
    return setters;
  }
}
