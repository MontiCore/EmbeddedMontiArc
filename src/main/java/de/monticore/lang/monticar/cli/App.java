package de.monticore.lang.monticar.cli;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.emam2wasm.EmamWasmSingleDirectoryCompiler;
import de.monticore.lang.monticar.setup.AutoSetup;
import de.se_rwth.commons.logging.Log;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.CommandLineRunner;
import org.springframework.boot.SpringApplication;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;

@Configuration
@ComponentScan
public class App implements CommandLineRunner {

  @Value("${setup:false}")
  private boolean setup;

  private AutoSetup autoSetup;

  private EmamWasmSingleDirectoryCompiler compiler;
  private ExpandedComponentInstanceSymbol model;

  @Autowired
  public App(AutoSetup autoSetup,
      EmamWasmSingleDirectoryCompiler compiler, ExpandedComponentInstanceSymbol model) {
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

    if (setup) {
      autoSetup.setup();
    } else {
      compiler.emam2wasm(model);
    }
  }

}
