package de.monticore.lang.monticar.generator.html;

import static de.monticore.lang.monticar.generator.GeneratorUtil.filterMultipleArrayPorts;
import static de.monticore.lang.monticar.generator.GeneratorUtil.getDimension;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.freemarker.TemplateProcessor;
import de.monticore.lang.monticar.generator.GeneratorUtil;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTType;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * This generator generates html files that can be used in combination with a
 * Javascript wrapper for a WebAssembly module.<p>
 * It creates an input field for every ingoing port of the models interface,
 * and a label for each outgoing port to display its value. Furthermore, it
 * creates a button to trigger the model execution and an area to write error
 * messages to.
 */
public class HtmlGenerator {

  private final TemplateProcessor templateProcessor;
  private final Function<ExpandedComponentInstanceSymbol, String> wasmNamingFunction;
  private final Function<ExpandedComponentInstanceSymbol, String> wrapperNamingFunction;

  public HtmlGenerator(TemplateProcessor templateProcessor,
      Function<ExpandedComponentInstanceSymbol, String> wasmNamingFunction,
      Function<ExpandedComponentInstanceSymbol, String> wrapperNamingFunction) {
    this.templateProcessor = templateProcessor;
    this.wasmNamingFunction = wasmNamingFunction;
    this.wrapperNamingFunction = wrapperNamingFunction;
  }

  /**
   * Generates a web interface for the ports given by the supplied model. For
   * each ingoing port of the models interface it creates an input text field
   * and it creates a label for each outgoing port to display its value. A
   * button triggers the execution of the model and error messages regarding
   * the execution will be displayed.
   *
   * @param model model to generate a HTML interface for
   * @throws IOException if an I/O exception occurs while writing the HTML file
   * @throws TemplateException if an exception occurs while processing the
   * template
   */
  public void generate(ExpandedComponentInstanceSymbol model)
      throws IOException, TemplateException {
    List<Port> inports = produceInports(model.getIncomingPorts());
    List<Port> outports = produceOutports(model.getOutgoingPorts());

    Map<String, Object> dataModel = new HashMap<>();
    dataModel.put("model", wasmNamingFunction.apply(model));
    dataModel.put("model_wrapper", wrapperNamingFunction.apply(model));
    dataModel.put("inports", inports);
    dataModel.put("outports", outports);

    templateProcessor.process(dataModel);
  }

  private List<Port> produceInports(Collection<PortSymbol> ports) {
    return producePorts(ports, GeneratorUtil::getSetterMethodName);
  }

  private List<Port> produceOutports(Collection<PortSymbol> ports) {
    return producePorts(ports, GeneratorUtil::getGetterMethodName);
  }

  private List<Port> producePorts(Collection<PortSymbol> ports,
      Function<PortSymbol, String> methodNameFunction) {
    return filterMultipleArrayPorts(ports).stream().map(
        p -> port(p.getNameWithoutArrayBracketPart(), methodNameFunction.apply(p),
            getType(ports, p))).collect(Collectors.toList());
  }

  private String getType(Collection<PortSymbol> ports, PortSymbol port) {
    MCASTTypeSymbolReference typeReference = (MCASTTypeSymbolReference) port.getTypeReference();
    ASTType astType = typeReference.getAstType();
    Optional<String> elementType;
    if (astType instanceof ASTCommonMatrixType) {
      ASTCommonMatrixType type = (ASTCommonMatrixType) astType;
      elementType = type.getElementType().getTElementType();
    } else if (astType instanceof ASTElementType) {
      ASTElementType type = (ASTElementType) astType;
      elementType = type.getTElementType();
    } else {
      throw new RuntimeException("Unexpected ASTType: " + astType);
    }
    String type = elementType.orElse("");

    String[] dimension = getDimension(ports, port);
    String dim = dimension.length > 0 ?
        "^{" + Arrays.stream(dimension).collect(Collectors.joining(", ")) + "}" : "";

    return type + dim;
  }

  private Port port(String name, String wrapperFunction, String type) {
    Port port = new Port();
    port.setName(name);
    port.setWrapperFunction(wrapperFunction);
    port.setType(type);
    return port;
  }
}
