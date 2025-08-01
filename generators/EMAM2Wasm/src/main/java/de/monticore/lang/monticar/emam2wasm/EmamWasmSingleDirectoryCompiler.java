/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.emam2wasm.cpp.CppStep;
import de.monticore.lang.monticar.emam2wasm.wasm.WasmStep;
import de.monticore.lang.monticar.emam2wasm.web.WebStep;
import java.nio.file.Path;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

/**
 * This class compiles EmbeddedMontiArc models to WebAssembly. It does so
 * without creating any intermediate directories.
 */
@Component
public class EmamWasmSingleDirectoryCompiler implements EmamWasmCompiler {

  private CppStep<ExpandedComponentInstanceSymbol> cppStep;
  private WasmStep wasmStep;
  private WebStep webStep;

  @Autowired
  public EmamWasmSingleDirectoryCompiler(
      CppStep<ExpandedComponentInstanceSymbol> cppStep,
      WasmStep wasmStep, WebStep webStep) {
    this.cppStep = cppStep;
    this.wasmStep = wasmStep;
    this.webStep = webStep;
  }

  @Override
  public Path emam2wasm(ExpandedComponentInstanceSymbol model) {
    Path cpp = cppStep.compileToCpp(model);
    Path wasm = wasmStep.compile(cpp);
    return webStep.generateWebInterface(model, wasm);
  }
}
