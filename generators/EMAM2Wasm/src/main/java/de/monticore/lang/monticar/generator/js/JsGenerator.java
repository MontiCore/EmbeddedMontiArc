/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.js;

import static de.monticore.lang.monticar.generator.GeneratorUtil.filterMultipleArrayPorts;
import static de.monticore.lang.monticar.generator.GeneratorUtil.getDimension;
import static de.monticore.lang.monticar.generator.GeneratorUtil.getGetterMethodName;
import static de.monticore.lang.monticar.generator.GeneratorUtil.getSetterMethodName;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.freemarker.TemplateProcessor;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTType;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import javax.measure.unit.Unit;
import org.jscience.mathematics.number.Rational;

public class JsGenerator {

  private final TemplateProcessor templateProcessor;

  public JsGenerator(TemplateProcessor templateProcessor) {
    this.templateProcessor = templateProcessor;
  }

  private static String getLowerBoundUnit(ASTRange range) {
    if (range.hasStartUnit()) {
      return format(range.getStartUnit());
    }
    return null;
  }

  private static String getUpperBoundUnit(ASTRange range) {
    if (range.hasEndUnit()) {
      return format(range.getEndUnit());
    }
    return null;
  }

  private static String getLowerBoundValue(ASTRange range) {
    if (!range.hasNoLowerLimit()) {
      Rational startValue = range.getStartValue();
      return startValue.toString();
    }
    return null;
  }

  private static String getUpperBoundValue(ASTRange range) {
    if (!range.hasNoUpperLimit()) {
      Rational startValue = range.getEndValue();
      return startValue.toString();
    }
    return null;
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

  private static String format(Unit<?> unit) {
    return unit.toString()
        .replaceAll("²", "^2")
        .replaceAll("³", "^3")
        .replaceAll("°", "deg")
        .replace('µ', 'u')
        .replace('·', '*');
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
      getter.setUnit(getRange(port).map(JsGenerator::getLowerBoundUnit).orElse(null));
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
      setter.setParameterName(port.getNameWithoutArrayBracketPart());
      setter.setDelegateMethodName(methodName);
      setter.setDimension(getDimension(rawIncomingPorts, port));

      Optional<ASTRange> rangeOpt = getRange(port);
      rangeOpt.ifPresent(range -> {
        setter.setLowerBoundUnit(getLowerBoundUnit(range));
        setter.setLowerBoundValue(getLowerBoundValue(range));
        setter.setUpperBoundUnit(getUpperBoundUnit(range));
        setter.setUpperBoundValue(getUpperBoundValue(range));
      });
      setters.add(setter);
    }
    return setters;
  }
}
