package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.simlang._ast.*;

import de.monticore.lang.montisim.weather._ast.ASTAlternativeInput;
import de.monticore.lang.montisim.weather._ast.ASTArtificialPhenomena;
import de.monticore.lang.montisim.weather._ast.ASTOpticalPhenomena;
import de.monticore.lang.montisim.weather._ast.ASTWeatherPhenomena;
import de.monticore.lang.montisim.weather.cocos.NumberUnit;
import de.monticore.lang.numberunit._ast.ASTUnitNumber;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.ImportStatement;

import de.se_rwth.commons.Names;
import de.monticore.lang.montisim.simlang.util.*;
import jline.internal.Log;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.Deque;

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
  public void visit(final ASTWeather node) {
    ArrayList<Weather> weathers = new ArrayList<>();
    if(node.getSingleWeather().isPresent()) {
      weathers.add(astToWeather(node.getSingleWeather().get()));
    } else if(node.getWeatherList().isPresent()) {
      for(ASTSingleWeather sw : node.getWeatherList().get().getSingleWeathers())
      weathers.add(astToWeather(sw));
    }

    final WeatherSymbol weatherSymbol = new WeatherSymbol("weather", weathers);
    addToScopeAndLinkWithNode(weatherSymbol, node);
  }
  private Weather astToWeather(ASTSingleWeather weather) {
    Weather ret;
    if(weather.getFixedWeather().isPresent()) {
      ret = new Weather(new FixedWeather(astToConcWeather(weather.getFixedWeather().get().getWeatherObj())));
    } else if(weather.getSequenceWeather().isPresent()) {
      ArrayList<ConcreteWeather> we = new ArrayList<>();
      ArrayList<NumberUnit> durs = new ArrayList<>();
      for(ASTWeatherObj wO : weather.getSequenceWeather().get().getWeatherObjs()){
        we.add(astToConcWeather(wO));
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
  private ConcreteWeather astToConcWeather(ASTWeatherObj obj) {
    obj.getWeatherAttributes().get(0).
    AlternativeInput temp = obj.getTemperatures().size() > 0 ? getUsedAlternative(obj.getTemperatures().get(0).getAlternativeInput()) : null;
    AlternativeInput humi = obj.getHumiditys().size() > 0 ? getUsedAlternative(obj.getHumiditys().get(0).getAlternativeInput()) : null;
    AlternativeInput press = obj.getPressures().size() > 0 ? getUsedAlternative(obj.getPressures().get(0).getAlternativeInput()) : null;
    AlternativeInput windS = obj.getWindStrengths().size() > 0 ? getUsedAlternative(obj.getWindStrengths().get(0).getAlternativeInput()) : null;
    AlternativeInput windD = obj.getWindDirections().size() > 0 ? getUsedAlternative(obj.getWindDirections().get(0).getAlternativeInput()) : null;
    AlternativeInput precA = obj.getPrecipitationAmounts().size() > 0 ? getUsedAlternative(obj.getPrecipitationAmounts().get(0).getAlternativeInput()) : null;

    Sight sight = null;
    if(obj.getSights().size() > 0) {
      sight = !obj.getSights().get(0).isUnlimited() ? new Sight(getUsedAlternative(obj.getSights().get(0).getAlternativeInput().get())) : new Sight();
    }


    SimLangEnums.CloudingTypes cloud = null;
    if(obj.getCloudings().size() > 0) {
      switch (obj.getCloudings().get(0).getCloudingType()) {
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
    }

    SimLangEnums.PrecipitationTypes precT = null;
    if(obj.getPrecipitationTypes().size() > 0) {
      switch (obj.getPrecipitationTypes().get(0).getPrecipitationType()) {
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
    }

    ArrayList<WeatherPhenomenaInstance> wPhen = new ArrayList<>();
    if(obj.getWeatherPhenomenas().size() > 0) {
      SimLangEnums.WeatherPhenomenas tmpPhen;
      Point2D.Float tmpCoord;
      for(ASTWeatherPhenomena astWP:obj.getWeatherPhenomenas()) {
        switch (astWP.getPhenomenaType()) {
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
        tmpCoord = astWP.coordinateIsPresent() ?
                new Point2D.Float(astWP.getCoordinate().get().getPosX().getNumber().get().floatValue(),
                                  astWP.getCoordinate().get().getPosY().getNumber().get().floatValue())
                : null;
        wPhen.add(new WeatherPhenomenaInstance(tmpPhen, tmpCoord));
      }
    }

    ArrayList<SimLangEnums.OpticalPhenomenas> oPhen = new ArrayList<>();
    if(obj.getOpticalPhenomenas().size() > 0) {
      SimLangEnums.OpticalPhenomenas tmpPhen;
      for(ASTOpticalPhenomena astWP:obj.getOpticalPhenomenas()) {
        switch (astWP.getPhenomenaType()) {
          case 0: tmpPhen = SimLangEnums.OpticalPhenomenas.RAINBOW; break;
          case 1: tmpPhen = SimLangEnums.OpticalPhenomenas.NORTHERN_LIGHTS; break;
          case 2: tmpPhen = SimLangEnums.OpticalPhenomenas.CIRCUMZENITHAL_ARC; break;
          case 3: tmpPhen = SimLangEnums.OpticalPhenomenas.ZODIACAL_LIGHTS; break;
          case 4: tmpPhen = SimLangEnums.OpticalPhenomenas.CREPUSCULAR_RAYS; break;
          case 5: tmpPhen = SimLangEnums.OpticalPhenomenas.MIRAGE; break;
          case 6: tmpPhen = SimLangEnums.OpticalPhenomenas.FOG_BOW; break;
          default: tmpPhen = SimLangEnums.OpticalPhenomenas.RAINBOW;
        }
        oPhen.add(tmpPhen);
      }
    }
    ArrayList<SimLangEnums.ArtificialPhenomena> aPhen = new ArrayList<>();
    if(obj.getArtificialPhenomenas().size() > 0) {
      SimLangEnums.ArtificialPhenomena tmpPhen;
      for(ASTArtificialPhenomena astWP:obj.getArtificialPhenomenas()) {
        switch (astWP.getPhenomenaType()) {
          case 0: tmpPhen = SimLangEnums.ArtificialPhenomena.CONTRAILS; break;
          case 1: tmpPhen = SimLangEnums.ArtificialPhenomena.SMOG; break;
          case 2: tmpPhen = SimLangEnums.ArtificialPhenomena.ROCKET_EXHAUST_TRAILS; break;
          default: tmpPhen = SimLangEnums.ArtificialPhenomena.CONTRAILS;
        }
        aPhen.add(tmpPhen);
      }
    }

    return new ConcreteWeather(temp, cloud, sight, precT, humi, press,
            windS, windD, precA, wPhen, oPhen, aPhen
    );
  }
  
  @Override
  public void visit(final ASTChannel node) {
    Log.info("visit channel"," -symtabcreator-");
    SimLangEnums.ChannelTypes type;
    switch (node.getChannelType()) {
      case 0: type = SimLangEnums.ChannelTypes.FIXED;
      break;
      case 1: type = SimLangEnums.ChannelTypes.BOUND;
      break;
      default: type = SimLangEnums.ChannelTypes.FIXED;
    }
    Area area;
    if(node.getAreas().get(0).isGlobal()) {
      area = new Area();
    } else if(node.getAreas().get(0).getRadius().isPresent()) {
      area = new Area(new Point2D.Float(node.getAreas().get(0).getPoint1().get().getPosX().getNumber().get().floatValue(),
              node.getAreas().get(0).getPoint1().get().getPosY().getNumber().get().floatValue()),
              new NumberUnit(node.getAreas().get(0).getRadius().get()));
    } else {
      area = new Area(new Point2D.Float(node.getAreas().get(0).getPoint1().get().getPosX().getNumber().get().floatValue(),
                                        node.getAreas().get(0).getPoint1().get().getPosY().getNumber().get().floatValue()),
              new Point2D.Float(node.getAreas().get(0).getPoint2().get().getPosX().getNumber().get().floatValue(),
                                node.getAreas().get(0).getPoint2().get().getPosY().getNumber().get().floatValue()));
    }
    Log.info("creating symbol "+node.getName()," -symtabcreator-");
    final ChannelSymbol symbol = new ChannelSymbol("channel",
            new Channel(type,
              node.getName(),
                    getUsedAlternative(node.getTransferRates().get(0).getAlternativeInput()),
                    getUsedAlternative(node.getLatencys().get(0).getAlternativeInput()),
                    getUsedAlternative(node.getOutages().get(0).getAlternativeInput()),
                    area
            ));
    Log.info("appending symbol "+node.getName()," -symtabcreator-");
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