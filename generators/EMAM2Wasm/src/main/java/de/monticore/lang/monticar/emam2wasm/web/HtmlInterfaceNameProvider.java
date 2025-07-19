/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.web;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;

import de.monticore.symboltable.Symbol;
import org.springframework.stereotype.Component;

@Component
public class HtmlInterfaceNameProvider implements InterfaceNameProvider {

  private static final String HTML_FILE_EXTENSION = "html";

  @Override
  public String getName(Symbol model) {
    return requiresNotNull(model).getName();
  }

  @Override
  public String getFileExtension() {
    return HTML_FILE_EXTENSION;
  }
}
