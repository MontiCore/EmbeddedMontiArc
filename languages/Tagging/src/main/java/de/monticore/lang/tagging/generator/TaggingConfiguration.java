/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.tagging.generator;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import de.se_rwth.commons.configuration.Configuration;
import de.se_rwth.commons.configuration.ConfigurationContributorChainBuilder;
import de.se_rwth.commons.configuration.DelegatingConfigurationContributor;

/**
 * Configuration of MontiArc generator.
 *
 */
public class TaggingConfiguration implements Configuration {

  public static final String CONFIGURATION_PROPERTY = "_configuration";
  public static final String DEFAULT_OUTPUT_DIRECTORY = "out";
  

  
  /**
   * The names of the specific MontiArc options used in this configuration.
   */
  public enum Options {
    
    MODELPATH("modelPath"), MODELPATH_SHORT("mp"),
    OUT("out"), OUT_SHORT("o"), HANDCODEDPATH("handcodedPath"), GRAMMARS("grammars"), MODEL("tagschema"), MODELS("tagschemas");
    
    String name;
    
    Options(String name) {
      this.name = name;
    }
    
    /**
     * @see Enum#toString()
     */
    @Override
    public String toString() {
      return this.name;
    }
    
  }
  
  
  private final Configuration configuration;
  
  /**
   * Factory method for {@link TaggingConfiguration}.
   */
  public static TaggingConfiguration withConfiguration(Configuration configuration) {
    return new TaggingConfiguration(configuration);
  }
  
  /**
   * Constructor for {@link TaggingConfiguration}
   */
  private TaggingConfiguration(Configuration internal) {
    
    // ConfigurationSystemPropertiesContributor systemPropertiesContributor =
    // ConfigurationSystemPropertiesContributor.withPrefix("montiarc");
    
    this.configuration = ConfigurationContributorChainBuilder.newChain()
        // .add(systemPropertiesContributor)
        .add(DelegatingConfigurationContributor.with(internal))
        .build();
  }
  
  /**
   * @see Configuration#getAllValues()
   */
  @Override
  public Map<String, Object> getAllValues() {
    return this.configuration.getAllValues();
  }
  
  /**
   * @see Configuration#getAllValuesAsStrings()
   */
  @Override
  public Map<String, String> getAllValuesAsStrings() {
    return this.configuration.getAllValuesAsStrings();
  }
  
  /**
   * @see Configuration#getAsBoolean(String)
   */
  @Override
  public Optional<Boolean> getAsBoolean(String key) {
    return this.configuration.getAsBoolean(key);
  }
  
  public Optional<Boolean> getAsBoolean(Enum<?> key) {
    return getAsBoolean(key.toString());
  }
  
  /**
   * @see Configuration#getAsBooleans(String)
   */
  @Override
  public Optional<List<Boolean>> getAsBooleans(String key) {
    return this.configuration.getAsBooleans(key);
  }
  
  public Optional<List<Boolean>> getAsBooleans(Enum<?> key) {
    return getAsBooleans(key.toString());
  }
  
  /**
   * @see Configuration#getAsDouble(String)
   */
  @Override
  public Optional<Double> getAsDouble(String key) {
    return this.configuration.getAsDouble(key);
  }
  
  public Optional<Double> getAsDouble(Enum<?> key) {
    return getAsDouble(key.toString());
  }
  
  /**
   * @see Configuration#getAsDoubles(String)
   */
  @Override
  public Optional<List<Double>> getAsDoubles(String key) {
    return this.configuration.getAsDoubles(key);
  }
  
  public Optional<List<Double>> getAsDoubles(Enum<?> key) {
    return getAsDoubles(key.toString());
  }
  
  /**
   * @see Configuration#getAsInteger(String)
   */
  @Override
  public Optional<Integer> getAsInteger(String key) {
    return this.configuration.getAsInteger(key);
  }
  
  public Optional<Integer> getAsInteger(Enum<?> key) {
    return getAsInteger(key.toString());
  }
  
  /**
   * @see Configuration#getAsIntegers(String)
   */
  @Override
  public Optional<List<Integer>> getAsIntegers(String key) {
    return this.configuration.getAsIntegers(key);
  }
  
  public Optional<List<Integer>> getAsIntegers(Enum<?> key) {
    return getAsIntegers(key.toString());
  }
  
  /**
   * @see Configuration#getAsString(String)
   */
  @Override
  public Optional<String> getAsString(String key) {
    return this.configuration.getAsString(key);
  }
  
  public Optional<String> getAsString(Enum<?> key) {
    return getAsString(key.toString());
  }
  
  /**
   * @see Configuration#getAsStrings(String)
   */
  @Override
  public Optional<List<String>> getAsStrings(String key) {
    return this.configuration.getAsStrings(key);
  }
  
  public Optional<List<String>> getAsStrings(Enum<?> key) {
    return getAsStrings(key.toString());
  }
  
  /**
   * @see Configuration#getValue(String)
   */
  @Override
  public Optional<Object> getValue(String key) {
    return this.configuration.getValue(key);
  }
  
  public Optional<Object> getValue(Enum<?> key) {
    return getValue(key.toString());
  }
  
  /**
   * @see Configuration#getValues(String)
   */
  @Override
  public Optional<List<Object>> getValues(String key) {
    return this.configuration.getValues(key);
  }
  
  public Optional<List<Object>> getValues(Enum<?> key) {
    return getValues(key.toString());
  }
  
//  /**
//   * Getter for the list of model path elements (files and directories) stored in this
//   * configuration.
//   * 
//   * @return list of model path files
//   */
//  public List<File> getModelPath() {
//    Optional<List<String>> modelPath = getAsStrings(Options.MODELPATH);
//    if (modelPath.isPresent()) {
//      return toFileList(modelPath.get());
//    }
//    modelPath = getAsStrings(Options.MODELPATH_SHORT);
//    if (modelPath.isPresent()) {
//      return toFileList(modelPath.get());
//    }
//    // default model path is empty
//    return Collections.emptyList();
//  }
  
  public List<File> getModelPath() {
    Optional<String> modelPath = getAsString(Options.MODELPATH);
    if(modelPath.isPresent()){
      Path mp = Paths.get(modelPath.get());
      return Arrays.asList(mp.toFile());
    }
    modelPath = getAsString(Options.MODELPATH_SHORT);
    if(modelPath.isPresent()){
      Path mp = Paths.get(modelPath.get());
      return Arrays.asList(mp.toFile());
    }
    return Collections.emptyList();
  }
  

  /**
   * Getter for the output directory stored in this configuration. A fallback
   * default is "out".
   * 
   * @return output directory file
   */
  public File getOut() {
    Optional<String> out = getAsString(Options.OUT);
    if (out.isPresent()) {
      return new File(out.get());
    }
    out = getAsString(Options.OUT_SHORT);
    if (out.isPresent()) {
      return new File(out.get());
    }
    // fallback default is "out"
    return new File(DEFAULT_OUTPUT_DIRECTORY);
  }
  
  
  /**
   * @param files as String names to convert
   * @return list of files by creating file objects from the Strings
   */
  protected static List<File> toFileList(List<String> files) {
    return files.stream().collect(Collectors.mapping(file -> new File(file), Collectors.toList()));
  }

  /**
   * @see Configuration#hasProperty(String)
   */
  @Override
  public boolean hasProperty(String key) {
   return this.configuration.hasProperty(key);
  }

  /**
   * Getter for model path stored in this configuration. As default it returns an empty String
   * @return path to model
   */
  public String getModel() {
    Optional<String> model = getAsString(Options.MODEL);
    if (model.isPresent()) {
      return model.get();
    }
    return "";
  }
  
  public List<String> getModels() {
      Optional<List<Object>> models = getValues(Options.MODELS);
      List<String> retModels = new ArrayList<String>();
      if(models.isPresent()){
        List<Object> objects = models.get();
        for(Object o : objects){
          if(o instanceof String){
            retModels.add((String)o);
          }
        }
      }
      return retModels;
  }
  
  public Optional<String> getHWCPath(){
    return getAsString(Options.HANDCODEDPATH);
  }
}
