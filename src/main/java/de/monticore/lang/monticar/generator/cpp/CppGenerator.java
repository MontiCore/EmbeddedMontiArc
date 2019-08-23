/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import static de.monticore.lang.monticar.generator.GeneratorUtil.filterMultipleArrayPorts;
import static de.monticore.lang.monticar.generator.GeneratorUtil.getGetterMethodName;
import static de.monticore.lang.monticar.generator.GeneratorUtil.getSetterMethodName;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.freemarker.TemplateProcessor;
import de.monticore.lang.monticar.generator.cpp.Method.Datatype;
import de.monticore.lang.monticar.generator.cpp.Method.Type;
import de.monticore.lang.monticar.generator.cpp.TypeConverter.CppTypes;
import de.monticore.lang.monticar.generator.cpp.TypeConverter.EmbeddedMontiArcTypes;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class CppGenerator {

  private static final String GETTER_PREFIX = "get";
  private static final String SETTER_PREFIX = "set";

  private final TemplateProcessor templateProcessor;

  public CppGenerator(TemplateProcessor templateProcessor) {
    this.templateProcessor = templateProcessor;
  }

  protected static String getCppClassName(String fullName) {
    return fullName.replace('.', '_');
  }

  protected static CppTypes getCppDataType(PortSymbol port) {
    return TypeConverter.getConversion(getMontiArcDataType(port));
  }

  protected static EmbeddedMontiArcTypes getMontiArcDataType(PortSymbol port) {
    return EmbeddedMontiArcTypes.get(port.getTypeReference().getName());
  }

  public void generate(ExpandedComponentInstanceSymbol symbol)
      throws IOException, TemplateException {
    Collection<PortSymbol> outports = filterMultipleArrayPorts(symbol.getOutgoingPorts());
    Collection<PortSymbol> inports = filterMultipleArrayPorts(symbol.getIncomingPorts());
    List<Getter> getters = produceGetters(outports);
    List<Setter> setters = produceSetters(inports);
    String mainClassName = getCppClassName(symbol.getFullName());

    Map<String, Object> dataModel = new HashMap<>();
    dataModel.put("getters", getters);
    dataModel.put("setters", setters);
    dataModel.put("main_class", mainClassName);

    templateProcessor.process(dataModel);
  }

  private List<Getter> produceGetters(Collection<PortSymbol> ports) {
    List<Getter> getters = new ArrayList<>();

    for (PortSymbol port : ports) {
      Type type;
      String name;
      if (port.isPartOfPortArray()) {
        name = port.getNameWithoutArrayBracketPart();
        type = Type.ARRAY;
      } else {
        name = port.getName();
        type = Type.SCALAR;
      }
      String methodName = getGetterMethodName(port);
      Datatype datatype =
          getCppDataType(port) == CppTypes.MATRIX ? Datatype.MATRIX : Datatype.PRIMITIVE;
      String variableName = name;

      Getter getter = new Getter(datatype, type, methodName, variableName);
      getters.add(getter);
    }
    return getters;
  }

  private List<Setter> produceSetters(Collection<PortSymbol> ports) {
    List<Setter> setters = new ArrayList<>();

    for (PortSymbol port : ports) {
      Type type;
      String name;
      if (port.isPartOfPortArray()) {
        name = port.getNameWithoutArrayBracketPart();
        type = Type.ARRAY;
      } else {
        name = port.getName();
        type = Type.SCALAR;
      }
      String methodName = getSetterMethodName(port);
      Datatype datatype =
          getCppDataType(port) == CppTypes.MATRIX ? Datatype.MATRIX : Datatype.PRIMITIVE;
      String variableName = name;
      String variableType = getCppDataType(port).getType();
      String parameterName = name;

      Setter setter = new Setter(datatype, type, methodName, variableName, variableType,
          parameterName);
      setters.add(setter);
    }
    return setters;
  }

}
