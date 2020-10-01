/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;
import de.monticore.lang.montisim.weather.symboltable.*;
import de.monticore.symboltable.ResolvingConfiguration;
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
    addResolvingFilter(new TimeResolvingFilter());
    addResolvingFilter(new MapOverlapResolvingFilter());
    addResolvingFilter(new MapSectorWidthResolvingFilter());
    addResolvingFilter(new MapSectorHeightResolvingFilter());
    addResolvingFilter(new MaxSectorUsersResolvingFilter());
    addResolvingFilter(new TimeoutResolvingFilter());
    addResolvingFilter(new GravityResolvingFilter());
    addResolvingFilter(new PedestrianDensityResolvingFilter());

    addResolvingFilter(new MapPathResolvingFilter());
    addResolvingFilter(new MapNameResolvingFilter());
    addResolvingFilter(new MapHeightResolvingFilter());

    addResolvingFilter(new PedestrianResolvingFilter());
    addResolvingFilter(new ExplicitVehicleResolvingFilter());
    addResolvingFilter(new PathedVehicleResolvingFilter());
    addResolvingFilter(new LTLVehicleResolvingFilter());
    addResolvingFilter(new RandomVehicleResolvingFilter());

    addResolvingFilter(new ChannelResolvingFilter());
    addResolvingFilter(new TransferRateResolvingFilter());
    addResolvingFilter(new LatencyResolvingFilter());
    addResolvingFilter(new OutageResolvingFilter());
    addResolvingFilter(new AreaResolvingFilter());

    addResolvingFilter(new WeatherResolvingFilter());
    addResolvingFilter(new TemperatureResolvingFilter());
    addResolvingFilter(new HumidityResolvingFilter());
    addResolvingFilter(new PressureResolvingFilter());
    addResolvingFilter(new WindStrengthResolvingFilter());
    addResolvingFilter(new WindDirectionResolvingFilter());
    addResolvingFilter(new PrecipitationTypeResolvingFilter());
    addResolvingFilter(new PrecipitationAmountResolvingFilter());
    addResolvingFilter(new CloudingResolvingFilter());
    addResolvingFilter(new SightResolvingFilter());
    addResolvingFilter(new WeatherPhenomenaResolvingFilter());
    addResolvingFilter(new OpticalPhenomenaResolvingFilter());
    addResolvingFilter(new ArtificialPhenomenaResolvingFilter());
  }
}
