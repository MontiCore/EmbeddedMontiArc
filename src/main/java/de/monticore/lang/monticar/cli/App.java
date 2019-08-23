/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cli;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.emam2wasm.EmamWasmSingleDirectoryCompiler;
import de.monticore.lang.monticar.setup.AutoSetup;
import de.se_rwth.commons.logging.Log;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.CommandLineRunner;
import org.springframework.boot.SpringApplication;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;
import org.springframework.lang.Nullable;

@Configuration
@ComponentScan
public class App implements CommandLineRunner {

  private EmamWasmSingleDirectoryCompiler compiler;
  private ExpandedComponentInstanceSymbol model;

  private AutoSetup autoSetup;

  @Autowired
  public App(@Nullable AutoSetup autoSetup,
      @Nullable EmamWasmSingleDirectoryCompiler compiler,
      @Nullable ExpandedComponentInstanceSymbol model) {
    this.autoSetup = autoSetup;
    this.compiler = compiler;
    this.model = model;
  }

  public static void main(String[] args) {
    SpringApplication.run(App.class, args);
  }

  @Override
  public void run(String... args) {
    Log.enableFailQuick(false);

    if (autoSetup == null && (compiler == null || model == null)) {
      System.out.println("Consider specifying a spring profile");
      return;
    }

    if (autoSetup != null) {
      autoSetup.setup();
    }
    if (compiler != null && model != null) {
      compiler.emam2wasm(model);
    }
  }
}
