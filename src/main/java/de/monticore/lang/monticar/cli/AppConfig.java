package de.monticore.lang.monticar.cli;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.emscripten.Emscripten;
import de.monticore.lang.monticar.emscripten.EmscriptenCommand;
import de.monticore.lang.monticar.emscripten.EmscriptenCommandBuilderFactory;
import de.monticore.lang.monticar.emscripten.Option;
import de.monticore.lang.monticar.emscripten.Shell;
import de.monticore.lang.monticar.freemarker.TemplateFactory;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.SymTabCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import freemarker.cache.ClassTemplateLoader;
import freemarker.cache.FileTemplateLoader;
import freemarker.cache.TemplateLoader;
import freemarker.template.Template;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.ComponentScan.Filter;
import org.springframework.context.annotation.Configuration;
import org.springframework.context.annotation.FilterType;
import org.springframework.context.annotation.Profile;

@Profile("compiler")
@Configuration
@ComponentScan(basePackages = {
    "de.monticore.lang.monticar.adapter",
    "de.monticore.lang.monticar.emam2wasm"},
    excludeFilters = @Filter(type = FilterType.REGEX,
        pattern = "de\\.monticore\\.lang\\.monticar\\.emam2wasm\\.model..*"))
public class AppConfig {

  private static final Path REGULAR_TEMPLATE_DIR = Paths.get("src/main/resources/ftl/cpp");
  private static final String JAR_TEMPLATE_DIR = "/ftl/cpp";
  private static final String TEMPLATE_NAME = "cpp.ftl";
  private static final String WINDOWS = "windows";

  @Value("${model}")
  private String modelFullName;

  @Value("${model-path}")
  private Path modelPath;

  @Value("${target:}")
  private Path target;

  @Value("${cpp-dir:.}")
  private Path cppDir;

  @Value("${wasm-dir:.}")
  private Path wasmDir;

  @Value("${emscripten:}")
  private String emscripten;

  @Value("#{'${include:}' ?: {}}")
  private Path[] includes;

  @Value("#{systemProperties['os.name'].toLowerCase()}")
  private String osName;

  @Bean
  public ExpandedComponentInstanceSymbol model(TaggingResolver taggingResolver) {
    return new Resolver(taggingResolver).getExpandedComponentInstanceSymbol(modelFullName);
  }

  @Bean
  public GeneratorCPP generator() {
    return new GeneratorCPP();
  }

  @Bean
  public Template template() throws IOException {
    TemplateLoader loader;
    if (runsInJar()) {
      loader = new ClassTemplateLoader(AppConfig.class, JAR_TEMPLATE_DIR);
    } else {
      loader = new FileTemplateLoader(REGULAR_TEMPLATE_DIR.toFile());
    }
    return new TemplateFactory(loader).getTemplate(TEMPLATE_NAME);
  }

  @Bean
  public TaggingResolver taggingResolver() {
    return new SymTabCreator(modelPath).createSymTabAndTaggingResolver();
  }

  @Bean
  public EmscriptenCommandBuilderFactory commandBuilderFactory() {
    EmscriptenCommandBuilderFactory commandBuilderFactory = new EmscriptenCommandBuilderFactory();
    commandBuilderFactory.setEmscripten(emscripten());
    for (Path include : includes) {
      commandBuilderFactory.include(include);
    }
    commandBuilderFactory.setStd("c++11");
    commandBuilderFactory.addOption(new Option("WASM", true));
    commandBuilderFactory.addOption(new Option("LINKABLE", true));
    commandBuilderFactory.addOption(new Option("EXPORT_ALL", true));
    commandBuilderFactory.addOption(new Option("ALLOW_MEMORY_GROWTH", true));
    commandBuilderFactory.setBind(true);
    return commandBuilderFactory;
  }

  @Bean
  public Emscripten emscripten() {
    Shell shell = osName.startsWith(WINDOWS) ? Shell.CMD : Shell.BASH;
    return new EmscriptenCommand(shell, emscripten);
  }

  @Bean
  public Path modelPath() {
    return modelPath;
  }

  @Bean
  public Path cppDir() {
    return target != null ? target : cppDir;
  }

  @Bean
  public Path wasmDir() {
    return target != null ? target : wasmDir;
  }

  private boolean runsInJar() {
    String classJar = AppConfig.class.getResource("AppConfig.class").toString();
    return classJar.startsWith("jar:");
  }
}
