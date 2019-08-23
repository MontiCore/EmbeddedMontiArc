/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cli;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.adapter.GeneratorCppWrapper;
import de.monticore.lang.monticar.emscripten.Emscripten;
import de.monticore.lang.monticar.emscripten.EmscriptenCommand;
import de.monticore.lang.monticar.emscripten.EmscriptenCommandBuilderFactory;
import de.monticore.lang.monticar.emscripten.EmscriptenConfigFileParser;
import de.monticore.lang.monticar.emscripten.Option;
import de.monticore.lang.monticar.emscripten.Shell;
import de.monticore.lang.monticar.freemarker.TemplateFactory;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.SymTabCreator;
import de.monticore.lang.monticar.util.TextFile;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import freemarker.cache.ClassTemplateLoader;
import freemarker.cache.FileTemplateLoader;
import freemarker.cache.MultiTemplateLoader;
import freemarker.cache.TemplateLoader;
import freemarker.template.Template;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.ComponentScan.Filter;
import org.springframework.context.annotation.Conditional;
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

  private static final Path REGULAR_TEMPLATE_DIR = Paths.get("src/main/resources/ftl");
  private static final Path REGULAR_CPP_TEMPLATE_DIR = REGULAR_TEMPLATE_DIR.resolve("cpp");
  private static final Path REGULAR_JS_TEMPLATE_DIR = REGULAR_TEMPLATE_DIR.resolve("js");
  private static final Path REGULAR_HTML_TEMPLATE_DIR = REGULAR_TEMPLATE_DIR.resolve("html");
  private static final String JAR_CPP_TEMPLATE_DIR = "/ftl/cpp";
  private static final String JAR_JS_TEMPLATE_DIR = "/ftl/js";
  private static final String JAR_HTML_TEMPLATE_DIR = "/ftl/html";
  private static final String CPP_TEMPLATE_NAME = "cpp.ftl";
  private static final String JS_TEMPLATE_NAME = "js.ftl";
  private static final String HTML_TEMPLATE_NAME = "html.ftl";
  private static final String EMSCRIPTEN_CONFIG_KEY = "EMSCRIPTEN_ROOT";

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

  @Value("${web-dir:.}")
  private Path webDir;

  @Value("${emscripten.execute.command:}")
  private String emscripten;

  @Value("#{systemProperties['user.home']}/.emscripten")
  private Path emscriptenConfig;

  @Value("#{'${include:}'.split(',') ?: {}}")
  private Path[] includes;

  @Value("#{'${library:}'.split(',') ?: {}}")
  private Path[] libraries;

  @Value("${option:WASM=1, LINKABLE=1, EXPORT_ALL=1, ALLOW_MEMORY_GROWTH=1}")
  private String[] options;

  @Value("${flag:std=c++11}")
  private String[] flags;

  @Value("${bind:true}")
  private boolean bind;

  @Value("${emscripten.execute.binary.name:emcc.bat}")
  private String emscriptenBinaryName;

  @Value("${algebraic-optimization:true}")
  private boolean useAlgebraicOptimization;

  @Value("${generate-tests:true}")
  private boolean generateTests;

  @Bean
  public OptionConverter optionConverter() {
    return new OptionConverter();
  }

  @Bean
  public ExpandedComponentInstanceSymbol model(TaggingResolver taggingResolver) {
    return new Resolver(taggingResolver).getExpandedComponentInstanceSymbol(modelFullName);
  }

  @Bean
  public GeneratorCPP generator() {
    return new GeneratorCPP();
  }

  @Bean
  public GeneratorCppWrapper generatorCppWrapper(GeneratorCPP generatorCPP, TaggingResolver symtab,
      Path modelPath) {
    GeneratorCppWrapper cppWrapper = new GeneratorCppWrapper(generatorCPP, symtab,
        modelPath);
    cppWrapper.setUseAlgebraicOptimization(useAlgebraicOptimization);
    cppWrapper.setGenerateTests(generateTests);
    return cppWrapper;
  }

  @Bean
  public Template cppTemplate(TemplateLoader templateLoader) throws IOException {
    return new TemplateFactory(templateLoader).getTemplate(CPP_TEMPLATE_NAME);
  }

  @Bean
  public Template jsTemplate(TemplateLoader templateLoader) throws IOException {
    return new TemplateFactory(templateLoader).getTemplate(JS_TEMPLATE_NAME);
  }

  @Bean
  public Template htmlTemplate(TemplateLoader templateLoader) throws IOException {
    return new TemplateFactory(templateLoader).getTemplate(HTML_TEMPLATE_NAME);
  }

  @Bean
  @Conditional(JarCondition.class)
  public TemplateLoader jarTemplateLoader() throws IOException {
    MultiTemplateLoader mtl;
    ClassTemplateLoader ctl1 = new ClassTemplateLoader(AppConfig.class, JAR_CPP_TEMPLATE_DIR);
    ClassTemplateLoader ctl2 = new ClassTemplateLoader(AppConfig.class, JAR_JS_TEMPLATE_DIR);
    ClassTemplateLoader ctl3 = new ClassTemplateLoader(AppConfig.class, JAR_HTML_TEMPLATE_DIR);
    mtl = new MultiTemplateLoader(new TemplateLoader[]{ctl1, ctl2, ctl3});
    return mtl;
  }

  @Bean
  @Conditional(DevCondition.class)
  public TemplateLoader devTemplateLoader() throws IOException {
    MultiTemplateLoader mtl;
    FileTemplateLoader ftl1 = new FileTemplateLoader(REGULAR_CPP_TEMPLATE_DIR.toFile());
    FileTemplateLoader ftl2 = new FileTemplateLoader(REGULAR_JS_TEMPLATE_DIR.toFile());
    FileTemplateLoader ftl3 = new FileTemplateLoader(REGULAR_HTML_TEMPLATE_DIR.toFile());
    mtl = new MultiTemplateLoader(new TemplateLoader[]{ftl1, ftl2, ftl3});
    return mtl;
  }

  @Bean
  public TaggingResolver taggingResolver() {
    return new SymTabCreator(modelPath).createSymTabAndTaggingResolver();
  }

  @Bean
  public EmscriptenCommandBuilderFactory commandBuilderFactory(Emscripten emscripten,
      Option[] options) {
    EmscriptenCommandBuilderFactory commandBuilderFactory = new EmscriptenCommandBuilderFactory();
    commandBuilderFactory.setEmscripten(emscripten);
    for (Path include : includes) {
      commandBuilderFactory.include(include);
    }
    for (Path library : libraries) {
      commandBuilderFactory.addLibrary(library);
    }
    for (Option option : options) {
      commandBuilderFactory.addOption(option);
    }
    for (String flag : flags) {
      commandBuilderFactory.addFlag(flag);
    }
    commandBuilderFactory.setBind(bind);
    return commandBuilderFactory;
  }

  @Bean
  @Conditional(OtherOSCondition.class)
  public Emscripten emscriptenOther() {
    return new EmscriptenCommand(Shell.BASH, emscripten);
  }

  @Bean
  @Conditional(WindowsCondition.class)
  public Emscripten emscriptenWindows() {
    if (emscripten == null || emscripten.isEmpty()) {
      if (Files.isReadable(emscriptenConfig)) {
        TextFile configFile = new TextFile(emscriptenConfig);
        EmscriptenConfigFileParser parser = new EmscriptenConfigFileParser(configFile);
        Map<String, String> configurations = parser.parseConfigurations();
        String emscriptenDirString = configurations.get(EMSCRIPTEN_CONFIG_KEY);
        Path emscriptenDir = Paths.get(emscriptenDirString);
        Path emscriptenBin = emscriptenDir.resolve(emscriptenBinaryName);
        return new EmscriptenCommand(Shell.CMD,
            emscriptenBin.toAbsolutePath().normalize().toString());
      } else {
        System.out.println("Please setup emscripten first by activating profile 'setup'");
      }
    }
    return new EmscriptenCommand(Shell.CMD, emscripten);
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

  @Bean
  public Path webDir() {
    return target != null ? target : webDir;
  }

  @Bean
  public Option[] options() {
    OptionConverter optionConverter = optionConverter();
    Option[] optionArray = new Option[options.length];
    for (int i = 0; i < options.length; i++) {
      optionArray[i] = optionConverter.convert(options[i]);
    }
    return optionArray;
  }
}
