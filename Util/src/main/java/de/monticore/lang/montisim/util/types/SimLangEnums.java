/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

public class SimLangEnums {
  public enum ChannelTypes {
    FIXED, BOUND
  }
  public enum SimulationTypes {
    FIXED, REALTIME, MAXFPS
  }

  public enum SimulationHeightModes {
    FLAT, RANDOM
  }

  public enum WeatherTypes {
    FIXED, SEQUENCE, RANDOM
  }

  public enum CloudingTypes {
    NONE, CIRROSTRATUS, ALTOSTRATUS, STRATUS, NIMBOSTRATUS, NOCTILUCENT, POLAR_STRATOSPHERIC,
    CIRRUS, CIRROCUMULUS, ALTOCUMULUS, STRATOCUMULUS, CUMULUS_HUMILIS, CUMULUS_MEDIOCRIS,
    CUMULUS_CONGESTUS, CUMULONIMBUS
  }

  public enum PrecipitationTypes {
    NONE, DRIZZLE, RAIN, FREEZING_DRIZZLE, FREEZING_RAIN, SNOW_RAIN, SNAIN, SNOW,
    SNOW_GRAINS, ICE_PELLETS, SLEET, HAIL, SNOW_PELLETS, GRAUPEL, ICE_CRYSTALS
  }

  public enum WeatherPhenomenas {
    FOG, ROPE_TORNADO, CONE_TORNADO, WEDGE_TORNADO, MULTI_VORTEX_TORNADO, LANDSPOUT,
    WATERSPOUT, GUSTNADO, DUST_DEVIL, STEAM_DEVIL, THUNDERSTORM
  }

  public enum OpticalPhenomenas {
    RAINBOW, NORTHERN_LIGHTS, CIRCUMZENITHAL_ARC, ZODIACAL_LIGHTS, CREPUSCULAR_RAYS, MIRAGE, FOG_BOW
  }

  public enum ArtificialPhenomena {
    CONTRAILS, SMOG, ROCKET_EXHAUST_TRAILS
  }
}
