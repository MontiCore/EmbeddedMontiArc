package de.monticore.lang.monticar.generator;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;
import static de.monticore.lang.monticar.contract.StringPrecondition.requiresNotBlank;

import com.google.common.annotations.VisibleForTesting;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonDimensionElement;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTType;
import java.util.ArrayList;
import java.util.Arrays;
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

  @VisibleForTesting
  public static String[] getDimension(Collection<PortSymbol> ports, PortSymbol port) {
    int arrayDimension = port.isPartOfPortArray() ?
        getArrayDimension(ports, port.getNameWithoutArrayBracketPart()) : 0;
    int[] matrixDimension = getMatrixDimension(port);
    return combineDimensions(arrayDimension, matrixDimension);
  }

  private static String[] combineDimensions(int arrayDimension, int[] matrixDimension) {
    if (arrayDimension > 0 && matrixDimension.length > 0) {
      int[] dimension = new int[matrixDimension.length + 1];
      dimension[0] = arrayDimension;
      System.arraycopy(matrixDimension, 0, dimension, 1, matrixDimension.length);
      return toStringArray(dimension);
    } else if (matrixDimension.length > 0) {
      return toStringArray(matrixDimension);
    } else if (arrayDimension > 0) {
      return new String[]{String.valueOf(arrayDimension)};
    } else {
      return new String[0];
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

  private static String[] toStringArray(int[] array) {
    return Arrays.stream(array).mapToObj(String::valueOf).toArray(String[]::new);
  }
}
