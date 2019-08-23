/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.web;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;

import de.monticore.symboltable.Symbol;
import org.springframework.stereotype.Component;

@Component
public class JsWrapperNameProvider implements WrapperNameProvider {

  private static final String JS_FILE_EXTENSION = "js";
  private static final String WRAPPER_SUFFIX = "wrapper";

  @Override
  public String getName(Symbol model) {
    return requiresNotNull(model).getName() + "_" + WRAPPER_SUFFIX;
  }

  @Override
  public String getFileExtension() {
    return JS_FILE_EXTENSION;
  }
}
