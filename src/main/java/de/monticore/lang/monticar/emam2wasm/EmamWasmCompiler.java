package de.monticore.lang.monticar.emam2wasm;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import java.nio.file.Path;

public interface EmamWasmCompiler {

  Path emam2wasm(ExpandedComponentInstanceSymbol model);
}
