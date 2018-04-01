package de.monticore.lang.monticar.generator.cpp;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;
import static de.monticore.lang.monticar.contract.StringPrecondition.requiresNotBlank;

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
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.springframework.util.StringUtils;

public class CppGenerator {

  private static final String GETTER_PREFIX = "get";
  private static final String SETTER_PREFIX = "set";

  private static final Pattern ARRAY_PATTERN = Pattern.compile("([a-zA-Z_$]\\w*)\\[\\d+\\]");

  private final TemplateProcessor templateProcessor;

  public CppGenerator(TemplateProcessor templateProcessor) {
    this.templateProcessor = templateProcessor;
  }

  protected static boolean isArray(PortSymbol port) {
    return isArray(port.getName());
  }

  protected static boolean isArray(String portName) {
    return ARRAY_PATTERN.matcher(portName).matches();
  }

  protected static String getArrayName(PortSymbol port) {
    return getArrayName(port.getName());
  }

  protected static String getArrayName(String portName) {
    Matcher matcher = ARRAY_PATTERN.matcher(portName);
    if (matcher.matches()) {
      return matcher.group(1);
    } else {
      throw new IllegalArgumentException(
          "Port " + portName + " is not part of a port array.");
    }
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

  protected static String getSetterMethodName(PortSymbol port) {
    return getSetterMethodName(requiresNotNull(port).getName());
  }

  protected static String getSetterMethodName(String portName) {
    return SETTER_PREFIX + StringUtils.capitalize(requiresNotBlank(portName));
  }

  protected static String getGetterMethodName(PortSymbol port) {
    return getGetterMethodName(requiresNotNull(port).getName());
  }

  protected static String getGetterMethodName(String portName) {
    return GETTER_PREFIX + StringUtils.capitalize(requiresNotBlank(portName));
  }

  public void generate(ExpandedComponentInstanceSymbol symbol)
      throws IOException, TemplateException {
    List<Getter> getters = produceGetters(symbol.getOutgoingPorts());
    List<Setter> setters = produceSetters(symbol.getIncomingPorts());
    String mainClassName = getCppClassName(symbol.getFullName());

    Map<String, Object> dataModel = new HashMap<>();
    dataModel.put("getters", getters);
    dataModel.put("setters", setters);
    dataModel.put("main_class", mainClassName);

    templateProcessor.process(dataModel);
  }

  private List<Getter> produceGetters(Collection<PortSymbol> ports) {
    List<Getter> getters = new ArrayList<>();

    Set<String> processedArrays = new HashSet<>();
    for (PortSymbol port : ports) {
      Type type;
      String name;
      if (isArray(port)) {
        String arrayName = getArrayName(port);

        if (processedArrays.contains(arrayName)) {
          continue;
        }

        processedArrays.add(arrayName);
        name = arrayName;
        type = Type.ARRAY;
      } else {
        name = port.getName();
        type = Type.SCALAR;
      }
      String methodName = GETTER_PREFIX + StringUtils.capitalize(name);
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

    Set<String> processedArrays = new HashSet<>();
    for (PortSymbol port : ports) {
      Type type;
      String name;
      if (isArray(port)) {
        String arrayName = getArrayName(port);

        if (processedArrays.contains(arrayName)) {
          continue;
        }

        processedArrays.add(arrayName);
        name = arrayName;
        type = Type.ARRAY;
      } else {
        name = port.getName();
        type = Type.SCALAR;
      }
      String methodName = SETTER_PREFIX + StringUtils.capitalize(name);
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
