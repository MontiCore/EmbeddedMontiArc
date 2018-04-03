package de.monticore.lang.monticar.generator;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;
import static de.monticore.lang.monticar.contract.StringPrecondition.requiresNotBlank;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import org.springframework.util.StringUtils;

public class GeneratorUtil {

  private static final String GETTER_PREFIX = "get";
  private static final String SETTER_PREFIX = "set";

  public static Collection<PortSymbol> filterMultipleArrayPorts(Collection<PortSymbol> ports) {
    List<PortSymbol> filteredPorts = new ArrayList<>();
    List<String> processedArrays = new ArrayList<>();
    for (PortSymbol port : ports) {
      if (port.isPartOfPortArray()) {
        String arrayName = port.getNameWithoutArrayBracketPart();
        if (processedArrays.contains(arrayName)) {
          continue;
        }
        processedArrays.add(arrayName);
      }
      filteredPorts.add(port);
    }
    return filteredPorts;
  }

  public static String getSetterMethodName(PortSymbol port) {
    return getSetterMethodName(requiresNotNull(port).getNameWithoutArrayBracketPart());
  }

  public static String getSetterMethodName(String portName) {
    return SETTER_PREFIX + StringUtils.capitalize(requiresNotBlank(portName));
  }

  public static String getGetterMethodName(PortSymbol port) {
    return getGetterMethodName(requiresNotNull(port).getNameWithoutArrayBracketPart());
  }

  public static String getGetterMethodName(String portName) {
    return GETTER_PREFIX + StringUtils.capitalize(requiresNotBlank(portName));
  }
}
