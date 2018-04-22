package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.simlang._ast.*;
import de.monticore.lang.montisim.util.types.*;

import de.monticore.lang.montisim.weather._ast.*;
import de.monticore.lang.montisim.weather.symboltable.*;
import de.monticore.lang.numberunit._ast.ASTUnitNumber;
import de.monticore.symboltable.*;

import de.se_rwth.commons.Names;
import jline.internal.Log;

import java.awt.geom.Point2D;
import java.util.*;
import java.util.stream.Collectors;

public class SimLangSymbolTableCreator extends SimLangSymbolTableCreatorTOP {
  
  public SimLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
    super(resolvingConfig, enclosingScope);
  }
  public SimLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
  }

  private Float nullOrFloat(Optional<ASTUnitNumber> opt) {
    if(opt.isPresent()) {
      return opt.get().getNumber().get().floatValue();
    }
    return null;
  }
  private Integer nullOrInteger(Optional<ASTUnitNumber> opt) {
    if(opt.isPresent()) {
      return opt.get().getNumber().get().intValue();
    }
    return null;
  }

  // Scope Symbols
  @Override
  public void visit(ASTSimLangCompilationUnit node) {
    String packageQualifiedName = Names.getQualifiedName(node.getPackage());
    List<ImportStatement> imports = node.getImportStatements()
            .stream()
            .map(imprt -> {
                String qualifiedImport = Names.getQualifiedName(imprt.getImportList());
                return new ImportStatement(qualifiedImport, imprt.isStar());
            })
            .collect(Collectors.toList());
    ArtifactScope artifactScope = new ArtifactScope(packageQualifiedName, imports);
    putOnStack(artifactScope);
  }
  @Override
  public void endVisit(ASTSimLangCompilationUnit node) {
    removeCurrentScope();
  }
  
  @Override
  public void visit(final ASTSimulation node) {
    final SimulationSymbol simSymbol = new SimulationSymbol(node.getName());
    addToScopeAndLinkWithNode(simSymbol, node);
  }

  @Override
  public void endVisit(final ASTSimulation node) {
    removeCurrentScope();
  }


  @Override
  public void visit(final ASTWeatherScope node) {
    MutableScope scope = new CommonScope();
    putOnStack(scope);
    setLinkBetweenSpannedScopeAndNode(scope, node);
  }
  @Override
  public void endVisit(final ASTWeatherScope node) {
    removeCurrentScope();
  }

  @Override
  public void visit(final ASTWeather node) {
    final WeatherSymbol weatherSymbol = new WeatherSymbol("weather", null);
    addToScopeAndLinkWithNode(weatherSymbol, node);
  }
  @Override
  public void endVisit(final ASTWeather node) {
    ArrayList<Weather> weathers = new ArrayList<>();
    if(node.getSingleWeather().isPresent()) {
      weathers.add(astToWeather(node.getSingleWeather().get()));
    } else if(node.getWeatherList().isPresent()) {
      for(ASTSingleWeather sw : node.getWeatherList().get().getSingleWeathers())
        weathers.add(astToWeather(sw));
    }
    ((WeatherSymbol)node.getSymbol().get()).setWeathers(weathers);
  }

  private Weather astToWeather(ASTSingleWeather weather) {
    Weather ret;
    if(weather.getFixedWeather().isPresent()) {
      ret = new Weather(new FixedWeather(resolveWeather(weather.getFixedWeather().get().getWeatherScope())));
    } else if(weather.getSequenceWeather().isPresent()) {
      ArrayList<ConcreteWeather> we = new ArrayList<>();
      ArrayList<NumberUnit> durs = new ArrayList<>();
      for(ASTWeatherScope wO : weather.getSequenceWeather().get().getWeatherScopes()){
        we.add(resolveWeather(wO));
      }
      for(ASTUnitNumber dur : weather.getSequenceWeather().get().getUnitNumbers()) {
        durs.add(new NumberUnit(dur));
      }
      ret = new Weather(new SequenceWeather(we, durs));
    } else {
      if(weather.getRandomWeather().get().getUnitNumber().isPresent()) {
        ret = new Weather(new RandomWeather(new NumberUnit(weather.getRandomWeather().get().getUnitNumber().get())));
      }
      else {
        ret = new Weather(new RandomWeather());
      }
    }
    return ret;
  }

  private ConcreteWeather resolveWeather(ASTWeatherScope node) {
    Scope scope = node.getSpannedScope().get();

    TemperatureSymbol tempS = scope.<TemperatureSymbol>resolve("temperature", TemperatureSymbol.KIND).orElse(null);
    AlternativeInput temperature = tempS == null ? null : tempS.getTemperature();


    HumiditySymbol humS = scope.<HumiditySymbol>resolve("humidity", HumiditySymbol.KIND).orElse(null);
    AlternativeInput humidity = humS == null ? null : humS.getHumidity();

    PressureSymbol presS = scope.<PressureSymbol>resolve("pressure", PressureSymbol.KIND).orElse(null);
    AlternativeInput pressure = presS == null ? null : presS.getPressure();

    WindStrengthSymbol windSS = scope.<WindStrengthSymbol>resolve("wind_strength", WindStrengthSymbol.KIND).orElse(null);
    AlternativeInput windStrength = windSS == null ? null : windSS.getWindStrength();

    WindDirectionSymbol windDS = scope.<WindDirectionSymbol>resolve("wind_direction", WindDirectionSymbol.KIND).orElse(null);
    AlternativeInput windDirection = windDS == null ? null : windDS.getWindDirection();

    PrecipitationTypeSymbol preTS = scope.<PrecipitationTypeSymbol>resolve("precipitation_type", PrecipitationTypeSymbol.KIND).orElse(null);
    SimLangEnums.PrecipitationTypes precipitationType = preTS == null ? null : preTS.getPrecipitationType();

    PrecipitationAmountSymbol preAS = scope.<PrecipitationAmountSymbol>resolve("precipitation_amount", PrecipitationAmountSymbol.KIND).orElse(null);
    AlternativeInput precipitationAmount = preAS == null ? null : preAS.getPrecipitationAmount();

    CloudingSymbol clouS = scope.<CloudingSymbol>resolve("clouding", CloudingSymbol.KIND).orElse(null);
    SimLangEnums.CloudingTypes clouding = clouS == null ? null : clouS.getClouding();

    SightSymbol sighS = scope.<SightSymbol>resolve("sight", SightSymbol.KIND).orElse(null);
    Sight sight = sighS == null ? null : sighS.getSight();

    Collection<WeatherPhenomenaSymbol> weatherPhen = scope.resolveMany("weather_phenomena", WeatherPhenomenaSymbol.KIND);
    ArrayList<WeatherPhenomenaInstance> wP = new ArrayList<>();
    for(WeatherPhenomenaSymbol sym : weatherPhen) {
      wP.add(sym.getWeatherPhenomena());
    }
    Collection<OpticalPhenomenaSymbol> opticalPhen = scope.resolveMany("optical_phenomena", OpticalPhenomenaSymbol.KIND);
    ArrayList<SimLangEnums.OpticalPhenomenas> oP = new ArrayList<>();
    for(OpticalPhenomenaSymbol sym : opticalPhen) {
      oP.add(sym.getOpticalPhenomena());
    }
    Collection<ArtificialPhenomenaSymbol> artificialPhen = scope.resolveMany("artificial_phenomena", ArtificialPhenomenaSymbol.KIND);
    ArrayList<SimLangEnums.ArtificialPhenomena> aP = new ArrayList<>();
    for(ArtificialPhenomenaSymbol sym : artificialPhen) {
      aP.add(sym.getArtificialPhenomena());
    }

    return new ConcreteWeather(temperature,humidity,pressure,windStrength,windDirection,precipitationType,precipitationAmount,clouding,sight,wP,oP,aP);
  }

  @Override
  public void visit(final ASTTemperature node) {
    final TemperatureSymbol symbol = new TemperatureSymbol("temperature", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTHumidity node) {
    final HumiditySymbol symbol = new HumiditySymbol("humidity", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTPressure node) {
    final PressureSymbol symbol = new PressureSymbol("pressure", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTWindStrength node) {
    final WindStrengthSymbol symbol = new WindStrengthSymbol("wind_strength", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTWindDirection node) {
    final WindDirectionSymbol symbol = new WindDirectionSymbol("wind_direction", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTPrecipitationAmount node) {
    final PrecipitationAmountSymbol symbol = new PrecipitationAmountSymbol("precipitation_amount", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }

  @Override
  public void visit(final ASTPrecipitationType node) {
    SimLangEnums.PrecipitationTypes precT;
    switch (node.getPrecipitationType()) {
      case 0: precT = SimLangEnums.PrecipitationTypes.NONE; break;
      case 1: precT = SimLangEnums.PrecipitationTypes.DRIZZLE; break;
      case 2: precT = SimLangEnums.PrecipitationTypes.RAIN; break;
      case 3: precT = SimLangEnums.PrecipitationTypes.FREEZING_DRIZZLE; break;
      case 4: precT = SimLangEnums.PrecipitationTypes.FREEZING_RAIN; break;
      case 5: precT = SimLangEnums.PrecipitationTypes.SNOW_RAIN; break;
      case 6: precT = SimLangEnums.PrecipitationTypes.SNAIN; break;
      case 7: precT = SimLangEnums.PrecipitationTypes.SNOW; break;
      case 8: precT = SimLangEnums.PrecipitationTypes.SNOW_GRAINS; break;
      case 9: precT = SimLangEnums.PrecipitationTypes.ICE_PELLETS; break;
      case 10: precT = SimLangEnums.PrecipitationTypes.SLEET; break;
      case 11: precT = SimLangEnums.PrecipitationTypes.HAIL; break;
      case 12: precT = SimLangEnums.PrecipitationTypes.SNOW_PELLETS; break;
      case 13: precT = SimLangEnums.PrecipitationTypes.GRAUPEL; break;
      case 14: precT = SimLangEnums.PrecipitationTypes.ICE_CRYSTALS; break;
      default: precT = SimLangEnums.PrecipitationTypes.NONE;
    }
    final PrecipitationTypeSymbol symbol = new PrecipitationTypeSymbol("precipitation_type", precT);
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTClouding node) {
    SimLangEnums.CloudingTypes cloud;
    switch (node.getCloudingType()) {
      case 0: cloud = SimLangEnums.CloudingTypes.NONE; break;
      case 1: cloud = SimLangEnums.CloudingTypes.CIRROSTRATUS; break;
      case 2: cloud = SimLangEnums.CloudingTypes.ALTOSTRATUS; break;
      case 3: cloud = SimLangEnums.CloudingTypes.STRATUS; break;
      case 4: cloud = SimLangEnums.CloudingTypes.NIMBOSTRATUS; break;
      case 5: cloud = SimLangEnums.CloudingTypes.NOCTILUCENT; break;
      case 6: cloud = SimLangEnums.CloudingTypes.POLAR_STRATOSPHERIC; break;
      case 7: cloud = SimLangEnums.CloudingTypes.CIRRUS; break;
      case 8: cloud = SimLangEnums.CloudingTypes.CIRROCUMULUS; break;
      case 9: cloud = SimLangEnums.CloudingTypes.ALTOCUMULUS; break;
      case 10: cloud = SimLangEnums.CloudingTypes.STRATOCUMULUS; break;
      case 11: cloud = SimLangEnums.CloudingTypes.CUMULUS_HUMILIS; break;
      case 12: cloud = SimLangEnums.CloudingTypes.CUMULUS_MEDIOCRIS; break;
      case 13: cloud = SimLangEnums.CloudingTypes.CUMULUS_CONGESTUS; break;
      case 14: cloud = SimLangEnums.CloudingTypes.CUMULONIMBUS; break;
      default: cloud = SimLangEnums.CloudingTypes.NONE;
    }
    final CloudingSymbol symbol = new CloudingSymbol("clouding", cloud);
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTSight node) {
    final SightSymbol symbol = node.isUnlimited() ? new SightSymbol("sight", new Sight()) : new SightSymbol("sight", new Sight(getUsedAlternative(node.getAlternativeInput().get())));
    addToScopeAndLinkWithNode(symbol, node);
  }

  @Override
  public void visit(final ASTWeatherPhenomena node) {
    SimLangEnums.WeatherPhenomenas tmpPhen;
    Point2D.Float tmpCoord;
      switch (node.getPhenomenaType()) {
        case 0: tmpPhen = SimLangEnums.WeatherPhenomenas.FOG; break;
        case 1: tmpPhen = SimLangEnums.WeatherPhenomenas.ROPE_TORNADO; break;
        case 2: tmpPhen = SimLangEnums.WeatherPhenomenas.CONE_TORNADO; break;
        case 3: tmpPhen = SimLangEnums.WeatherPhenomenas.WEDGE_TORNADO; break;
        case 4: tmpPhen = SimLangEnums.WeatherPhenomenas.MULTI_VORTEX_TORNADO; break;
        case 5: tmpPhen = SimLangEnums.WeatherPhenomenas.LANDSPOUT; break;
        case 6: tmpPhen = SimLangEnums.WeatherPhenomenas.WATERSPOUT; break;
        case 7: tmpPhen = SimLangEnums.WeatherPhenomenas.GUSTNADO; break;
        case 9: tmpPhen = SimLangEnums.WeatherPhenomenas.DUST_DEVIL; break;
        case 10: tmpPhen = SimLangEnums.WeatherPhenomenas.STEAM_DEVIL; break;
        case 11: tmpPhen = SimLangEnums.WeatherPhenomenas.THUNDERSTORM; break;
        default: tmpPhen = SimLangEnums.WeatherPhenomenas.FOG;
      }
      tmpCoord = node.coordinateIsPresent() ?
              new Point2D.Float(node.getCoordinate().get().getPosX().getNumber().get().floatValue(),
                      node.getCoordinate().get().getPosY().getNumber().get().floatValue())
              : null;

    final WeatherPhenomenaSymbol symbol = new WeatherPhenomenaSymbol("weather_phenomena", new WeatherPhenomenaInstance(tmpPhen, tmpCoord));
    addToScopeAndLinkWithNode(symbol, node);
  }

  @Override
  public void visit(final ASTOpticalPhenomena node) {
    SimLangEnums.OpticalPhenomenas tmpPhen;
    switch (node.getPhenomenaType()) {
      case 0: tmpPhen = SimLangEnums.OpticalPhenomenas.RAINBOW; break;
      case 1: tmpPhen = SimLangEnums.OpticalPhenomenas.NORTHERN_LIGHTS; break;
      case 2: tmpPhen = SimLangEnums.OpticalPhenomenas.CIRCUMZENITHAL_ARC; break;
      case 3: tmpPhen = SimLangEnums.OpticalPhenomenas.ZODIACAL_LIGHTS; break;
      case 4: tmpPhen = SimLangEnums.OpticalPhenomenas.CREPUSCULAR_RAYS; break;
      case 5: tmpPhen = SimLangEnums.OpticalPhenomenas.MIRAGE; break;
      case 6: tmpPhen = SimLangEnums.OpticalPhenomenas.FOG_BOW; break;
      default: tmpPhen = SimLangEnums.OpticalPhenomenas.RAINBOW;
    }
    final OpticalPhenomenaSymbol symbol = new OpticalPhenomenaSymbol("optical_phenomena", tmpPhen);
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTArtificialPhenomena node) {
    SimLangEnums.ArtificialPhenomena tmpPhen;
    switch (node.getPhenomenaType()) {
      case 0:
        tmpPhen = SimLangEnums.ArtificialPhenomena.CONTRAILS;
        break;
      case 1:
        tmpPhen = SimLangEnums.ArtificialPhenomena.SMOG;
        break;
      case 2:
        tmpPhen = SimLangEnums.ArtificialPhenomena.ROCKET_EXHAUST_TRAILS;
        break;
      default:
        tmpPhen = SimLangEnums.ArtificialPhenomena.CONTRAILS;
    }
    final ArtificialPhenomenaSymbol symbol = new ArtificialPhenomenaSymbol("artificial_phenomena", tmpPhen);
    addToScopeAndLinkWithNode(symbol, node);
  }

  /*
   * Channel Symbolmanagement
   *
   */

  @Override
  public void visit(final ASTChannel node) {
    final ChannelSymbol symbol = new ChannelSymbol("channel");
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void endVisit(final ASTChannel node) {
    SimLangEnums.ChannelTypes type;
    switch (node.getChannelType()) {
      case 0: type = SimLangEnums.ChannelTypes.FIXED;
        break;
      case 1: type = SimLangEnums.ChannelTypes.BOUND;
        break;
      default: type = SimLangEnums.ChannelTypes.FIXED;
    }
    AlternativeInput transferRate = node.getSpannedScope().get().<TransferRateSymbol>resolve("transfer_rate", TransferRateSymbol.KIND).get().getTransferRate();
    AlternativeInput latency = node.getSpannedScope().get().<LatencySymbol>resolve("latency", LatencySymbol.KIND).get().getLatency();
    AlternativeInput outage = node.getSpannedScope().get().<OutageSymbol>resolve("outage", OutageSymbol.KIND).get().getOutage();
    Area area = node.getSpannedScope().get().<AreaSymbol>resolve("area", AreaSymbol.KIND).get().getArea();
    Channel channel = new Channel(type,node.getName(),transferRate,latency,outage,area);

    ((ChannelSymbol)node.getSymbol().get()).setChannel(channel);
    removeCurrentScope();
  }

  @Override
  public void visit(final ASTTransferRate node) {
    final TransferRateSymbol symbol = new TransferRateSymbol("transfer_rate", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTLatency node) {
    final LatencySymbol symbol = new LatencySymbol("latency", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTOutage node) {
    final OutageSymbol symbol = new OutageSymbol("outage", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTArea node) {
    Area area;
    if(node.isGlobal()) {
      area = new Area();
    } else if(node.getRadius().isPresent()) {
      area = new Area(new Point2D.Float(node.getPoint1().get().getPosX().getNumber().get().floatValue(),
                                        node.getPoint1().get().getPosY().getNumber().get().floatValue()),
                      new NumberUnit(node.getRadius().get()));
    } else {
      area = new Area(new Point2D.Float(node.getPoint1().get().getPosX().getNumber().get().floatValue(),
              node.getPoint1().get().getPosY().getNumber().get().floatValue()),
              new Point2D.Float(node.getPoint2().get().getPosX().getNumber().get().floatValue(),
                      node.getPoint2().get().getPosY().getNumber().get().floatValue()));
    }
    final AreaSymbol symbol = new AreaSymbol("area", area);
    addToScopeAndLinkWithNode(symbol, node);
  }

  // Standard Symbols

  private AlternativeInput getUsedAlternative(ASTAlternativeInput node) {
    if (node.unitNumberIsPresent()) {
      return new AlternativeInput(new NumberUnit(node.getUnitNumber().get()));
    }
    else if(node.unitNumberListIsPresent()) {
      ArrayList<NumberUnit> tmplist = new ArrayList<>();
      for(ASTUnitNumber un : node.getUnitNumberList().get().getUnitNumbers()) {
        tmplist.add(new NumberUnit(un));
      }
      return new AlternativeInput(tmplist);
    }
    else if(node.rangeIsPresent()) {
      NumberUnit start = new NumberUnit(node.getRange().get().getStartValue().floatValue(), node.getRange().get().getStartUnit().toString());
      NumberUnit step = new NumberUnit(node.getRange().get().getStepValue().floatValue(), node.getRange().get().getStepUnit().toString());
      NumberUnit end = new NumberUnit(node.getRange().get().getEndValue().floatValue(), node.getRange().get().getEndUnit().toString());
      return new AlternativeInput(new Range(start, step, end));
    }
    else {
      Log.error("Error: unhandled alternative input.");
      return null;
    }
  }

  //Simulation
  public void visit(final ASTSimulationRenderFrequency node) {
    final SimulationRenderFrequencySymbol symbol = new SimulationRenderFrequencySymbol("sim_render_frequency", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTSimulationLoopFrequency node) {
    final SimulationLoopFrequencySymbol symbol = new SimulationLoopFrequencySymbol("sim_loop_frequency", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTSimulationDuration node) {
    final SimulationDurationSymbol symbol = new SimulationDurationSymbol("sim_duration", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTSimulationType node) {
    final SimulationTypeSymbol symbol;
    switch(node.getSimType()) {
      case 0:
        symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.FIXED);
        break;
      case 1:
        symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.REALTIME);
        break;
      case 2:
        symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.MAXFPS);
        break;
      default:
        //Log.error("Something broke while visiting sim_type node.");
        symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.FIXED);
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTTime node) {
    final TimeSymbol symbol;
    ArrayList<Time> list = new ArrayList<>();
    if(node.getSingleTime().isPresent()) {
      Time val = new Time(node.getSingleTime().get().getHours().getNumber().get().intValue(),
                          node.getSingleTime().get().getMinutes().getNumber().get().intValue(),
                          nullOrInteger(node.getSingleTime().get().getSeconds()),
                          nullOrInteger(node.getSingleTime().get().getMilliseconds()));
      list.add(val);
      symbol = new TimeSymbol("time", list);
    }
    else {
      for(ASTSingleTime ele : node.getTimeList().get().getSingleTimes()) {
        list.add(new Time(ele.getHours().getNumber().get().intValue(),
                          ele.getMinutes().getNumber().get().intValue(),
                          nullOrInteger(ele.getSeconds()),
                          nullOrInteger(ele.getMilliseconds())));
      }
      symbol = new TimeSymbol("time", list);
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapPath node) {
    final MapPathSymbol symbol = new MapPathSymbol("map_path", node.getMapPath());
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapName node) {
    final MapNameSymbol symbol = new MapNameSymbol("map_name", node.getMapName(),node.getFileFormat());
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapHeight node) {
    final MapHeightSymbol symbol;
    if(node.getCustomHeight().isPresent()) {
      symbol = new MapHeightSymbol("map_height", new MapHeight(node.getCustomHeight()));
    }
    else {
      switch(node.getHeightMode()) {
        case 0:
          symbol = new MapHeightSymbol("map_height", new MapHeight(SimLangEnums.SimulationHeightModes.FLAT));
          break;
        case 1:
          symbol = new MapHeightSymbol("map_height", new MapHeight(SimLangEnums.SimulationHeightModes.RANDOM));
          break;
        default:
          //Log.error("Something went wrong when visiting height mode node");
          symbol = new MapHeightSymbol("map_height", new MapHeight(SimLangEnums.SimulationHeightModes.FLAT));
      }
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapOverlap node) {
    final MapOverlapSymbol symbol = new MapOverlapSymbol("map_overlap", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapSectorWidth node) {
    final MapSectorWidthSymbol symbol = new MapSectorWidthSymbol("map_sector_width", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapSectorHeight node) {
    final MapSectorHeightSymbol symbol = new MapSectorHeightSymbol("map_sector_height", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMaxSectorUsers node) {
    final MaxSectorUsersSymbol symbol = new MaxSectorUsersSymbol("max_sector_users", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTTimeout node) {
    final TimeoutSymbol symbol = new TimeoutSymbol("timeout", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTGravity node) {
    final GravitySymbol symbol = new GravitySymbol("gravity", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTPedestrianDensity node) {
    final PedestrianDensitySymbol symbol = new PedestrianDensitySymbol("pedestrian_density", getUsedAlternative(node.getAlternativeInput()));
    addToScopeAndLinkWithNode(symbol, node);
  }

  public void visit(ASTPedestrians node) {
    final PedestrianSymbol symbol = new PedestrianSymbol("pedestrian",
            new Pedestrian(node.getStartX().getNumber().get().floatValue(),
                    node.getStartY().getNumber().get().floatValue(),
                    node.getDestX().getNumber().get().floatValue(),
                    node.getDestY().getNumber().get().floatValue(),
                    nullOrFloat(node.getStartZ()),
                    nullOrFloat(node.getDestZ())
                    ));
    addToScopeAndLinkWithNode(symbol, node);
  }

  public void visit(ASTExplicitVehicle node) {
    final ExplicitVehicleSymbol symbol = new ExplicitVehicleSymbol("explicit_vehicle",
            new ExplicitVehicle(node.getName(),
                    node.getStartX().getNumber().get().floatValue(),
                    node.getStartY().getNumber().get().floatValue(),
                    node.getDestX().getNumber().get().floatValue(),
                    node.getDestY().getNumber().get().floatValue(),
                    node.getStartRot().getNumber().get().floatValue(),
                    nullOrFloat(node.getDestZ())
    ));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(ASTPathedVehicle node) {
    final PathedVehicleSymbol symbol = new PathedVehicleSymbol("pathed_vehicle",
            new PathedVehicle(node.getStartX().getNumber().get().floatValue(),
                    node.getStartY().getNumber().get().floatValue(),
                    new NumberUnit(node.getSpawnRadius()),
                    node.getDestX().getNumber().get().floatValue(),
                    node.getDestY().getNumber().get().floatValue(),
                    new NumberUnit(node.getDestRadius()),
                    nullOrFloat(node.getAmount())
            ));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(ASTRandomVehicle node) {
    final RandomVehicleSymbol symbol = new RandomVehicleSymbol("random_vehicle",
              new RandomVehicle(node.getAmount().getNumber().get().floatValue(),
                      nullOrFloat(node.getStartX()),
                      nullOrFloat(node.getStartY()),
                      nullOrFloat(node.getDestX()),
                      nullOrFloat(node.getDestY())
                      ));
    addToScopeAndLinkWithNode(symbol, node);
  }
}