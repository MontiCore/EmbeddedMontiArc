/*
 * Copyright (c) 2014 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.montisim.simlang._symboltable;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
//import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.MutableScope;

import java.util.Optional;

public class SimLangLang extends SimLangLanguage {
  public static final String FILE_ENDING = "sim";
  
  public SimLangLang() {
    super("SimLang Language", FILE_ENDING);

    initResolvingFilters();
    setModelNameCalculator(new SimLangModelNameCalculator());
  }
  
  @Override
  public Optional<SimLangSymbolTableCreator> getSymbolTableCreator(
      ResolvingConfiguration resolvingConfiguration, MutableScope enclosingScope) {
    return Optional.of(new SimLangSymbolTableCreator(resolvingConfiguration, enclosingScope));
  }

  @Override
  protected SimLangModelLoader provideModelLoader() {
    return new SimLangModelLoader(this);
  }

  @Override
  protected void initResolvingFilters() {
    addResolvingFilter(new SimulationResolvingFilter());
    addResolvingFilter(new SimulationRenderFrequencyResolvingFilter());
    addResolvingFilter(new SimulationLoopFrequencyResolvingFilter());
    addResolvingFilter(new SimulationDurationResolvingFilter());
    addResolvingFilter(new SimulationTypeResolvingFilter());
    //addResolvingFilter(new WeatherResolvingFilter());
    //addResolvingFilter(new TemperatureResolvingFilter());
    //addResolvingFilter(new CloudingResolvingFilter());
    //addResolvingFilter(new SightResolvingFilter());
    //addResolvingFilter(new PrecipitationtypeResolvingFilter());
    //addResolvingFilter(new HumidityResolvingFilter());
    //addResolvingFilter(new PressureResolvingFilter());
    //addResolvingFilter(new WindstrengthResolvingFilter());
    //addResolvingFilter(new WinddirectionResolvingFilter());
    //addResolvingFilter(new PrecipitationAmountResolvingFilter());
    //addResolvingFilter(new WeatherPhenomenaResolvingFilter());
    //addResolvingFilter(new OpticalPhenomenaResolvingFilter());
    //addResolvingFilter(new ArtificialPhenomenaResolvingFilter());
    addResolvingFilter(new TimeResolvingFilter());
    addResolvingFilter(new MapPathResolvingFilter());
    addResolvingFilter(new MapNameResolvingFilter());
    addResolvingFilter(new MapHeightResolvingFilter());
    addResolvingFilter(new MapOverlapResolvingFilter());
    addResolvingFilter(new MapSectorWidthResolvingFilter());
    addResolvingFilter(new MapSectorHeightResolvingFilter());
    addResolvingFilter(new MaxSectorUsersResolvingFilter());
    addResolvingFilter(new TimeoutResolvingFilter());
    addResolvingFilter(new GravityResolvingFilter());
    addResolvingFilter(new PedestrianDensityResolvingFilter());
    //addResolvingFilter(new PedestrianResolvingFilter());
    addResolvingFilter(new ExplicitVehicleResolvingFilter());
    //addResolvingFilter(new PathedVehicleResolvingFilter());
    //addResolvingFilter(new RandomVehicleResolvingFilter());
    //addResolvingFilter(new ChannelResolvingFilter());
    //addResolvingFilter(new TransferrateResolvingFilter());
    //addResolvingFilter(new LatencyResolvingFilter());
    //addResolvingFilter(new OutageResolvingFilter());
    //addResolvingFilter(new AreaResolvingFilter());
  }
}
