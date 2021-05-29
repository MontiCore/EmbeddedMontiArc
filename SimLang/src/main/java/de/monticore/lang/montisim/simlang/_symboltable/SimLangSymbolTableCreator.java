/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.simlang.SimLangTool;
import de.monticore.lang.montisim.simlang._ast.*;
import de.monticore.lang.montisim.util.types.*;

import de.monticore.lang.montisim.weather._ast.*;
import de.monticore.lang.montisim.weather.symboltable.*;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.*;

import de.se_rwth.commons.Names;
import jline.internal.Log;

import java.awt.geom.Point2D;
import java.util.*;
import java.util.stream.Collectors;

import de.monticore.lang.montisim.simlang._ast.ASTConstantsSimLang;
import de.monticore.lang.montisim.weather._ast.ASTConstantsWeather;

public class SimLangSymbolTableCreator extends SimLangSymbolTableCreatorTOP {

    public SimLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public SimLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    private Float nullOrFloat(Optional<ASTNumberWithUnit> opt) {
        if (opt.isPresent()) {
            return opt.get().getNumber().get().floatValue();
        }
        return null;
    }

    private Integer nullOrInteger(Optional<ASTNumberWithUnit> opt) {
        if (opt.isPresent()) {
            return opt.get().getNumber().get().intValue();
        }
        return null;
    }

    // Scope Symbols
    @Override
    public void visit(ASTSimLangCompilationUnit node) {
        String packageQualifiedName = Names.getQualifiedName(node.getPackageList());
        List<ImportStatement> imports = node.getImportStatementList()
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
        SimLangTool.checkDefaultCoCos(node);
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
        if (node.isPresentSingleWeather()) {
            weathers.add(astToWeather(node.getSingleWeather()));
        } else if (node.isPresentWeatherList()) {
            for (ASTSingleWeather sw : node.getWeatherList().getSingleWeatherList())
                weathers.add(astToWeather(sw));
        }
        ((WeatherSymbol) node.getSymbol().get()).setWeathers(weathers);
    }

    private Weather astToWeather(ASTSingleWeather weather) {
        Weather ret;
        if (weather.isPresentFixedWeather()) {
            ret = new Weather(new FixedWeather(resolveWeather(weather.getFixedWeather().getWeatherScope())));
        } else if (weather.isPresentSequenceWeather()) {
            ArrayList<ConcreteWeather> we = new ArrayList<>();
            ArrayList<NumberUnit> durs = new ArrayList<>();
            for (ASTWeatherScope wO : weather.getSequenceWeather().getWeatherScopeList()) {
                we.add(resolveWeather(wO));
            }
            for (ASTNumberWithUnit dur : weather.getSequenceWeather().getNumberWithUnitList()) {
                durs.add(new NumberUnit(dur));
            }
            ret = new Weather(new SequenceWeather(we, durs));
        } else {
            if (weather.getRandomWeather().getNumberWithUnitOpt().isPresent()) {
                ret = new Weather(new RandomWeather(new NumberUnit(weather.getRandomWeather().getNumberWithUnit())));
            } else {
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
        for (WeatherPhenomenaSymbol sym : weatherPhen) {
            wP.add(sym.getWeatherPhenomena());
        }
        Collection<OpticalPhenomenaSymbol> opticalPhen = scope.resolveMany("optical_phenomena", OpticalPhenomenaSymbol.KIND);
        ArrayList<SimLangEnums.OpticalPhenomenas> oP = new ArrayList<>();
        for (OpticalPhenomenaSymbol sym : opticalPhen) {
            oP.add(sym.getOpticalPhenomena());
        }
        Collection<ArtificialPhenomenaSymbol> artificialPhen = scope.resolveMany("artificial_phenomena", ArtificialPhenomenaSymbol.KIND);
        ArrayList<SimLangEnums.ArtificialPhenomena> aP = new ArrayList<>();
        for (ArtificialPhenomenaSymbol sym : artificialPhen) {
            aP.add(sym.getArtificialPhenomena());
        }

        return new ConcreteWeather(temperature, humidity, pressure, windStrength, windDirection, precipitationType, precipitationAmount, clouding, sight, wP, oP, aP);
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
            case ASTConstantsWeather.NONE:
                precT = SimLangEnums.PrecipitationTypes.NONE;
                break;
            case ASTConstantsWeather.DRIZZLE:
                precT = SimLangEnums.PrecipitationTypes.DRIZZLE;
                break;
            case ASTConstantsWeather.RAIN:
                precT = SimLangEnums.PrecipitationTypes.RAIN;
                break;
            case ASTConstantsWeather.FREEZINGDRIZZLE:
                precT = SimLangEnums.PrecipitationTypes.FREEZING_DRIZZLE;
                break;
            case ASTConstantsWeather.FREEZINGRAIN:
                precT = SimLangEnums.PrecipitationTypes.FREEZING_RAIN;
                break;
            case ASTConstantsWeather.SNOWRAIN:
                precT = SimLangEnums.PrecipitationTypes.SNOW_RAIN;
                break;
            case ASTConstantsWeather.SNAIN:
                precT = SimLangEnums.PrecipitationTypes.SNAIN;
                break;
            case ASTConstantsWeather.SNOW:
                precT = SimLangEnums.PrecipitationTypes.SNOW;
                break;
            case ASTConstantsWeather.SNOWGRAINS:
                precT = SimLangEnums.PrecipitationTypes.SNOW_GRAINS;
                break;
            case ASTConstantsWeather.ICEPELLETS:
                precT = SimLangEnums.PrecipitationTypes.ICE_PELLETS;
                break;
            case ASTConstantsWeather.SLEET:
                precT = SimLangEnums.PrecipitationTypes.SLEET;
                break;
            case ASTConstantsWeather.HAIL:
                precT = SimLangEnums.PrecipitationTypes.HAIL;
                break;
            case ASTConstantsWeather.SNOWPELLETS:
                precT = SimLangEnums.PrecipitationTypes.SNOW_PELLETS;
                break;
            case ASTConstantsWeather.GRAUPEL:
                precT = SimLangEnums.PrecipitationTypes.GRAUPEL;
                break;
            case ASTConstantsWeather.ICECRYSTALS:
                precT = SimLangEnums.PrecipitationTypes.ICE_CRYSTALS;
                break;
            default:
                precT = SimLangEnums.PrecipitationTypes.NONE;
        }
        final PrecipitationTypeSymbol symbol = new PrecipitationTypeSymbol("precipitation_type", precT);
        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void visit(final ASTClouding node) {
        SimLangEnums.CloudingTypes cloud;
        switch (node.getCloudingType()) {
            case ASTConstantsWeather.NONE:
                cloud = SimLangEnums.CloudingTypes.NONE;
                break;
            case ASTConstantsWeather.CIRROSTRATUS:
                cloud = SimLangEnums.CloudingTypes.CIRROSTRATUS;
                break;
            case ASTConstantsWeather.ALTOSTRATUS:
                cloud = SimLangEnums.CloudingTypes.ALTOSTRATUS;
                break;
            case ASTConstantsWeather.STRATUS:
                cloud = SimLangEnums.CloudingTypes.STRATUS;
                break;
            case ASTConstantsWeather.NIMBOSTRATUS:
                cloud = SimLangEnums.CloudingTypes.NIMBOSTRATUS;
                break;
            case ASTConstantsWeather.NOCTILUCENT:
                cloud = SimLangEnums.CloudingTypes.NOCTILUCENT;
                break;
            case ASTConstantsWeather.POLARSTRATOSPHERIC:
                cloud = SimLangEnums.CloudingTypes.POLAR_STRATOSPHERIC;
                break;
            case ASTConstantsWeather.CIRRUS:
                cloud = SimLangEnums.CloudingTypes.CIRRUS;
                break;
            case ASTConstantsWeather.CIRROCUMULUS:
                cloud = SimLangEnums.CloudingTypes.CIRROCUMULUS;
                break;
            case ASTConstantsWeather.ALTOCUMULUS:
                cloud = SimLangEnums.CloudingTypes.ALTOCUMULUS;
                break;
            case ASTConstantsWeather.STRATOCUMULUS:
                cloud = SimLangEnums.CloudingTypes.STRATOCUMULUS;
                break;
            case ASTConstantsWeather.CUMULUSHUMILIS:
                cloud = SimLangEnums.CloudingTypes.CUMULUS_HUMILIS;
                break;
            case ASTConstantsWeather.CUMULUSMEDIOCRIS:
                cloud = SimLangEnums.CloudingTypes.CUMULUS_MEDIOCRIS;
                break;
            case ASTConstantsWeather.CUMULUSCONGESTUS:
                cloud = SimLangEnums.CloudingTypes.CUMULUS_CONGESTUS;
                break;
            case ASTConstantsWeather.CUMULONIMBUS:
                cloud = SimLangEnums.CloudingTypes.CUMULONIMBUS;
                break;
            default:
                cloud = SimLangEnums.CloudingTypes.NONE;
        }
        final CloudingSymbol symbol = new CloudingSymbol("clouding", cloud);
        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void visit(final ASTSight node) {
        final SightSymbol symbol = node.isUnlimited() ? new SightSymbol("sight", new Sight()) : new SightSymbol("sight", new Sight(getUsedAlternative(node.getAlternativeInput())));
        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void visit(final ASTWeatherPhenomena node) {
        SimLangEnums.WeatherPhenomenas tmpPhen;
        Point2D.Float tmpCoord;
        switch (node.getPhenomenaType()) {
            case ASTConstantsWeather.FOG:
                tmpPhen = SimLangEnums.WeatherPhenomenas.FOG;
                break;
            case ASTConstantsWeather.ROPETORNADO:
                tmpPhen = SimLangEnums.WeatherPhenomenas.ROPE_TORNADO;
                break;
            case ASTConstantsWeather.CONETORNADO:
                tmpPhen = SimLangEnums.WeatherPhenomenas.CONE_TORNADO;
                break;
            case ASTConstantsWeather.WEDGETORNADO:
                tmpPhen = SimLangEnums.WeatherPhenomenas.WEDGE_TORNADO;
                break;
            case ASTConstantsWeather.MULTIVORTEXTORNADO:
                tmpPhen = SimLangEnums.WeatherPhenomenas.MULTI_VORTEX_TORNADO;
                break;
            case ASTConstantsWeather.LANDSPOUT:
                tmpPhen = SimLangEnums.WeatherPhenomenas.LANDSPOUT;
                break;
            case ASTConstantsWeather.WATERSPOUT:
                tmpPhen = SimLangEnums.WeatherPhenomenas.WATERSPOUT;
                break;
            case ASTConstantsWeather.GUSTNADO:
                tmpPhen = SimLangEnums.WeatherPhenomenas.GUSTNADO;
                break;
            case ASTConstantsWeather.DUSTDEVIL:
                tmpPhen = SimLangEnums.WeatherPhenomenas.DUST_DEVIL;
                break;
            case ASTConstantsWeather.STEAMDEVIL:
                tmpPhen = SimLangEnums.WeatherPhenomenas.STEAM_DEVIL;
                break;
            case ASTConstantsWeather.THUNDERSTORM:
                tmpPhen = SimLangEnums.WeatherPhenomenas.THUNDERSTORM;
                break;
            default:
                tmpPhen = SimLangEnums.WeatherPhenomenas.FOG;
        }
        tmpCoord = node.isPresentCoordinate() ?
                new Point2D.Float(node.getCoordinate().getPosX().getNumber().get().floatValue(),
                        node.getCoordinate().getPosY().getNumber().get().floatValue())
                : null;

        final WeatherPhenomenaSymbol symbol = new WeatherPhenomenaSymbol("weather_phenomena", new WeatherPhenomenaInstance(tmpPhen, tmpCoord));
        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void visit(final ASTOpticalPhenomena node) {
        SimLangEnums.OpticalPhenomenas tmpPhen;
        switch (node.getPhenomenaType()) {
            case ASTConstantsWeather.RAINBOW:
                tmpPhen = SimLangEnums.OpticalPhenomenas.RAINBOW;
                break;
            case ASTConstantsWeather.NORTHERNLIGHTS:
                tmpPhen = SimLangEnums.OpticalPhenomenas.NORTHERN_LIGHTS;
                break;
            case ASTConstantsWeather.CIRCUMZENITHALARC:
                tmpPhen = SimLangEnums.OpticalPhenomenas.CIRCUMZENITHAL_ARC;
                break;
            case ASTConstantsWeather.ZODIACALLIGHT:
                tmpPhen = SimLangEnums.OpticalPhenomenas.ZODIACAL_LIGHTS;
                break;
            case ASTConstantsWeather.CREPUSCULARRAYS:
                tmpPhen = SimLangEnums.OpticalPhenomenas.CREPUSCULAR_RAYS;
                break;
            case ASTConstantsWeather.MIRAGE:
                tmpPhen = SimLangEnums.OpticalPhenomenas.MIRAGE;
                break;
            case ASTConstantsWeather.FOGBOW:
                tmpPhen = SimLangEnums.OpticalPhenomenas.FOG_BOW;
                break;
            default:
                tmpPhen = SimLangEnums.OpticalPhenomenas.RAINBOW;
        }
        final OpticalPhenomenaSymbol symbol = new OpticalPhenomenaSymbol("optical_phenomena", tmpPhen);
        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void visit(final ASTArtificialPhenomena node) {
        SimLangEnums.ArtificialPhenomena tmpPhen;
        switch (node.getPhenomenaType()) {
            case ASTConstantsWeather.CONTRAILS:
                tmpPhen = SimLangEnums.ArtificialPhenomena.CONTRAILS;
                break;
            case ASTConstantsWeather.SMOG:
                tmpPhen = SimLangEnums.ArtificialPhenomena.SMOG;
                break;
            case ASTConstantsWeather.ROCKETEXHAUSTTRAILS:
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
            case ASTConstantsSimLang.FIXED:
                type = SimLangEnums.ChannelTypes.FIXED;
                break;
            case ASTConstantsSimLang.BOUND:
                type = SimLangEnums.ChannelTypes.BOUND;
                break;
            default:
                type = SimLangEnums.ChannelTypes.FIXED;
        }
        AlternativeInput transferRate = node.getSpannedScope().get().<TransferRateSymbol>resolve("transfer_rate", TransferRateSymbol.KIND).get().getTransferRate();
        AlternativeInput latency = node.getSpannedScope().get().<LatencySymbol>resolve("latency", LatencySymbol.KIND).get().getLatency();
        AlternativeInput outage = node.getSpannedScope().get().<OutageSymbol>resolve("outage", OutageSymbol.KIND).get().getOutage();
        Area area = node.getSpannedScope().get().<AreaSymbol>resolve("area", AreaSymbol.KIND).get().getArea();
        Channel channel = new Channel(type, node.getName(), transferRate, latency, outage, area);

        ((ChannelSymbol) node.getSymbol().get()).setChannel(channel);
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
        if (node.isGlobal()) {
            area = new Area();
        } else if (node.isPresentRadius()) {
            area = new Area(new Point2D.Float(node.getPoint1().getPosX().getNumber().get().floatValue(),
                    node.getPoint1().getPosY().getNumber().get().floatValue()),
                    new NumberUnit(node.getRadius()));
        } else {
            area = new Area(new Point2D.Float(node.getPoint1().getPosX().getNumber().get().floatValue(),
                    node.getPoint1().getPosY().getNumber().get().floatValue()),
                    new Point2D.Float(node.getPoint2().getPosX().getNumber().get().floatValue(),
                            node.getPoint2().getPosY().getNumber().get().floatValue()));
        }
        final AreaSymbol symbol = new AreaSymbol("area", area);
        addToScopeAndLinkWithNode(symbol, node);
    }

    // Standard Symbols

    private AlternativeInput getUsedAlternative(ASTAlternativeInput node) {
        if (node.isPresentNumberWithUnit()) {
            return new AlternativeInput(new NumberUnit(node.getNumberWithUnit()));
        } else if (node.isPresentNumberWithUnitList()) {
            ArrayList<NumberUnit> tmplist = new ArrayList<>();
            for (ASTNumberWithUnit un : node.getNumberWithUnitList().getNumberWithUnitList()) {
                tmplist.add(new NumberUnit(un));
            }
            return new AlternativeInput(tmplist);
        } else if (node.isPresentRange()) {
            NumberUnit start = new NumberUnit(node.getRange().getStartValue().floatValue(), node.getRange().getStartUnit().toString());
            NumberUnit step = new NumberUnit(node.getRange().getStepValue().floatValue(), node.getRange().getStepUnit().toString());
            NumberUnit end = new NumberUnit(node.getRange().getEndValue().floatValue(), node.getRange().getEndUnit().toString());
            return new AlternativeInput(new Range(start, step, end));
        } else {
            Log.warn("Error: unhandled alternative input.");
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
        switch (node.getSimType()) {
            case ASTConstantsSimLang.FIXED:
                symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.FIXED);
                break;
            case ASTConstantsSimLang.REALTIME:
                symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.REALTIME);
                break;
            case ASTConstantsSimLang.MAXFPS:
                symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.MAXFPS);
                break;
            default:
                //Log.warn("Something broke while visiting sim_type node.");
                symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.FIXED);
        }
        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void visit(final ASTTime node) {
        final TimeSymbol symbol;
        ArrayList<Time> list = new ArrayList<>();
        if (node.isPresentSingleTime()) {
            Time val = new Time(node.getSingleTime().getHours().getNumber().get().intValue(),
                    node.getSingleTime().getMinutes().getNumber().get().intValue(),
                    nullOrInteger(node.getSingleTime().getSecondsOpt()),
                    nullOrInteger(node.getSingleTime().getMillisecondsOpt()));
            list.add(val);
            symbol = new TimeSymbol("time", list);
        } else {
            for (ASTSingleTime ele : node.getTimeList().getSingleTimeList()) {
                list.add(new Time(ele.getHours().getNumber().get().intValue(),
                        ele.getMinutes().getNumber().get().intValue(),
                        nullOrInteger(ele.getSecondsOpt()),
                        nullOrInteger(ele.getMillisecondsOpt())));
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
        final MapNameSymbol symbol = new MapNameSymbol("map_name", node.getMapName(), node.getFileFormat());
        addToScopeAndLinkWithNode(symbol, node);
    }

    public void visit(final ASTMapHeight node) {
        final MapHeightSymbol symbol;
        if (node.isPresentCustomHeight()) {
            symbol = new MapHeightSymbol("map_height", new MapHeight(node.getCustomHeightOpt()));
        } else {
            switch (node.getHeightMode()) {
                case ASTConstantsSimLang.FLAT:
                    symbol = new MapHeightSymbol("map_height", new MapHeight(SimLangEnums.SimulationHeightModes.FLAT));
                    break;
                case ASTConstantsSimLang.RANDOM:
                    symbol = new MapHeightSymbol("map_height", new MapHeight(SimLangEnums.SimulationHeightModes.RANDOM));
                    break;
                default:
                    //Log.warn("Something went wrong when visiting height mode node");
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
                new Pedestrian(node.getStartLat().getNumber().get().floatValue(),
                        node.getStartLong().getNumber().get().floatValue(),
                        node.getDestLat().getNumber().get().floatValue(),
                        node.getDestLong().getNumber().get().floatValue(),
                        nullOrFloat(node.getStartZOpt()),
                        nullOrFloat(node.getDestZOpt())
                ));
        addToScopeAndLinkWithNode(symbol, node);
    }

    public void visit(final ASTLTLVehicle node) {
        LTLVehicle vehicle = new LTLVehicle();

        // extract vehicle settings from corresponding ast elements
        node.getVehicleSettingList().forEach(elem -> {
            if (elem instanceof ASTPath) {
                List<double[]> path = handlePath((ASTPath) elem);
                vehicle.setPath(path);
            } else if (elem instanceof ASTGoalList) {
                List<LTLVehicle.Goal> goals = handleGoalList((ASTGoalList) elem);
                vehicle.setGoals(Optional.of(goals));
            } else if (elem instanceof ASTPlatoon) {
                Optional<Double> size = ((ASTPlatoon) elem).getSize().getNumber();
                size.ifPresent(aDouble -> vehicle.setPlatoonSize(Optional.of(aDouble.intValue())));
            }
        });

        final LTLVehicleSymbol symbol = new LTLVehicleSymbol("ltl_vehicle", vehicle);
        addToScopeAndLinkWithNode(symbol, node);
    }

    private List<LTLVehicle.Goal> handleGoalList(ASTGoalList goalList) {
        return goalList.streamGoals().map(astGoal ->
                new LTLVehicle.Goal(
                        astGoal.getLTLOperator(),
                        astGoal.getMetricName(),
                        astGoal.getComparator(),
                        new NumberUnit(astGoal.getTarget())
                )
        ).collect(Collectors.toList());
    }

    private List<double[]> handlePath(ASTPath path) {
        List<double[]> ret = new ArrayList<>();
        ret.add(new double[]{
                path.getStartLat().getNumber().get(),
                path.getStartLong().getNumber().get(),
                path.getStartAlt().getNumber().get(),
        });
        for (int i = 0; i < path.getDestLatList().size(); i++) {
            ret.add(new double[]{
                    path.getDestLat(i).getNumber().get(),
                    path.getDestLong(i).getNumber().get(),
                    path.getDestAlt(i).getNumber().get()
            });
        }
        return ret;
    }

    @Override
    public void endVisit(ASTLTLVehicle node) {
        if (!node.getCarModelOpt().isPresent()) return;
        CarModelSymbolReference cms = (CarModelSymbolReference) node.getCarModel().getSymbolOpt().get();
        LTLVehicleSymbol symbol = (LTLVehicleSymbol) node.getSymbolOpt().get();
        symbol.getVehicle().setCarContainer(cms.getReferencedSymbol().getCarContainer());
    }

    // todo: make this a scope
    public void visit(ASTExplicitVehicle node) {
        final ExplicitVehicleSymbol symbol = new ExplicitVehicleSymbol("explicit_vehicle",
                new ExplicitVehicle(node.getName(),
                        node.getStartLat().getNumber().get().floatValue(),
                        node.getStartLong().getNumber().get().floatValue(),
                        node.getDestLat().getNumber().get().floatValue(),
                        node.getDestLong().getNumber().get().floatValue(),
                        node.getStartRot().getNumber().get().floatValue(),
                        nullOrFloat(node.getDestZOpt())
                ));
        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void endVisit(ASTExplicitVehicle node) {
        //super.endVisit(ast); // this calls removeScope()
        if (node.isPresentCarModel()) {
            CarModelSymbolReference cms = (CarModelSymbolReference) node.getCarModel().getSymbolOpt().get();
            ExplicitVehicleSymbol symbol = (ExplicitVehicleSymbol) node.getSymbolOpt().get();
            symbol.getVehicle().setCarContainer(cms.getReferencedSymbol().getCarContainer());
            // System.out.println("linked " + node.getName() + " and " + cms.getReferencedSymbol().getCarContainer().getMass());
        }
    }

    public void visit(ASTPathedVehicle node) {
        final PathedVehicleSymbol symbol = new PathedVehicleSymbol("pathed_vehicle",
                new PathedVehicle(node.getStartLat().getNumber().get().floatValue(),
                        node.getStartLong().getNumber().get().floatValue(),
                        new NumberUnit(node.getSpawnRadius()),
                        node.getDestLat().getNumber().get().floatValue(),
                        node.getDestLong().getNumber().get().floatValue(),
                        new NumberUnit(node.getDestRadius()),
                        nullOrFloat(node.getAmountOpt())
                ));
        addToScopeAndLinkWithNode(symbol, node);
    }

    public void visit(ASTRandomVehicle node) {
        final RandomVehicleSymbol symbol = new RandomVehicleSymbol("random_vehicle",
                new RandomVehicle(node.getAmount().getNumber().get().floatValue(),
                        nullOrFloat(node.getStartLatOpt()),
                        nullOrFloat(node.getStartLongOpt()),
                        nullOrFloat(node.getDestLatOpt()),
                        nullOrFloat(node.getDestLongOpt())
                ));
        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void visit(ASTCarModel node) {
        final CarModelSymbolReference symbol = new CarModelSymbolReference(node.getName(), currentScope().get());
        addToScopeAndLinkWithNode(symbol, node);
    }

}
