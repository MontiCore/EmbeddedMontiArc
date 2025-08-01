/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;
import static de.monticore.lang.monticar.contract.StringPrecondition.requiresNotBlank;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonDimensionElement;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTType;
import de.monticore.symboltable.Symbol;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import org.springframework.util.StringUtils;

/**
 * Class for methods commonly used by generator classes.
 */
public class GeneratorUtil {

  private static final String GETTER_PREFIX = "get";
  private static final String SETTER_PREFIX = "set";
  private static final String BOOLEAN_TYPE = "B";
  private static final String NATURAL_NUMBER_TYPE = "N";
  private static final String INTEGER_NUMBER_TYPE = "Z";
  private static final String RATIONAL_NUMBER_TYPE = "Q";
  private static final String COMPLEX_NUMBER_TYPE = "C";

  /**
   * Takes a collection of {@link PortSymbol}s and filters out ports that
   * belong to an array except for one. The order of the ports in the supplied
   * collection remains otherwise unchanged.<p>
   * It is guaranteed that the first port of an array as returned by the
   * collections iterator is put into the returned collection. All subsequent
   * ports belonging to this array will be discarded.<p>
   * Two ports are determined to belong to the same port array if their names
   * match disregarding the trailing square brackets.
   *
   * @param ports a collection of ports
   * @return collection that contains only one port for each port array and is
   * otherwise unchanged from the supplied collection
   */
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

  /**
   * Returns the method name to be used when generating a setter method for the
   * supplied port.
   *
   * @param port {@code PortSymbol}
   * @return method name for the supplied port
   */
  public static String getSetterMethodName(PortSymbol port) {
    return getSetterMethodName(requiresNotNull(port).getNameWithoutArrayBracketPart());
  }

  /**
   * Returns the method name to be used when generating a setter method for a
   * {@link PortSymbol} with the supplied name.
   *
   * @param portName the name of a {@code PortSymbol}
   * @return method name for the supplied port name
   */
  public static String getSetterMethodName(String portName) {
    return SETTER_PREFIX + StringUtils.capitalize(requiresNotBlank(portName));
  }

  /**
   * Returns the method name to be used when generating a getter method for the
   * supplied port.
   *
   * @param port {@code PortSymbol}
   * @return method name for the supplied port
   */
  public static String getGetterMethodName(PortSymbol port) {
    return getGetterMethodName(requiresNotNull(port).getNameWithoutArrayBracketPart());
  }

  /**
   * Returns the method name to be used when generating a getter method for a
   * {@link PortSymbol} with the supplied name.
   *
   * @param portName the name of a {@code PortSymbol}
   * @return method name for the supplied port name
   */
  public static String getGetterMethodName(String portName) {
    return GETTER_PREFIX + StringUtils.capitalize(requiresNotBlank(portName));
  }

  /**
   * Returns the component name for the supplied {@code Symbol}. This name
   * always starts with an capital letter.
   *
   * @param model {@code Symbol}
   * @return component name for the supplied symbol
   */
  public static String getComponentName(Symbol model) {
    return StringUtils.capitalize(requiresNotNull(model).getName());
  }

  /**
   * Tries to determine the supplied {@code PortSymbol}'s type. The type
   * includes the number set, any defined range and corresponding units.
   *
   * @param port {@code PortSymbol}
   * @return the port's type
   * @throws RuntimeException if it failed to determine the port's type
   */
  public static String getType(PortSymbol port) {
    MCASTTypeSymbolReference typeReference = (MCASTTypeSymbolReference) port.getTypeReference();
    ASTType astType = typeReference.getAstType();
    if (astType instanceof ASTCommonMatrixType) {
      return matrixType((ASTCommonMatrixType) astType);
    } else if (astType instanceof ASTElementType) {
      ASTElementType type = (ASTElementType) astType;
      Optional<String> elementType = type.getTElementType();
      return elementType.orElse(typeReference.getName());
    } else {
      throw new RuntimeException("Unexpected ASTType: " + astType);
    }
  }

  private static String matrixType(ASTCommonMatrixType type) {
    ASTElementType elementType = type.getElementType();
    Optional<String> tElementTypeOpt = elementType.getTElementType();
    if (tElementTypeOpt.isPresent()) {
      return tElementTypeOpt.get();
    } else {
      if (elementType.isIsBoolean()) {
        return BOOLEAN_TYPE;
      } else if (elementType.isIsNatural()) {
        return NATURAL_NUMBER_TYPE;
      } else if (elementType.isIsWholeNumberNumber()) {
        return INTEGER_NUMBER_TYPE;
      } else if (elementType.isIsRational()) {
        return RATIONAL_NUMBER_TYPE;
      } else if (elementType.isIsComplex()) {
        return COMPLEX_NUMBER_TYPE;
      } else {
        throw new RuntimeException("Unable to determine type from " + type);
      }
    }
  }

  /**
   * Returns a string array representing the dimension of the supplied port.
   * The entries in this array are strings of integer numbers. The dimension
   * is a combination of the port's potential matrix dimension and array size.
   * In case the supplied port is defined as an array of matrices, a string
   * array with the array and matrix dimension is returned. The size of the
   * port array will be in position 0 of the returned array.
   * In case the supplied port is either defined as a matrix or belongs to an
   * array, the respective dimension is returned.
   * Otherwise, a string array of size 0 is returned.<p>
   * A collection of all ports of the component that the supplied port belongs
   * to is needed to determine a potential array size.
   *
   * @param ports all ports directly belonging to the component of the
   * supplied port
   * @param port PortSymbol to determine the dimension for
   * @return dimension of the port or an empty array
   */
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
