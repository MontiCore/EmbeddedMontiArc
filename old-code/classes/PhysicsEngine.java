/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

/**
 * Physics calculations for simulation
 */
public class PhysicsEngine {

    public static double calcFrictionCoefficient(StreetPavements streetPavement, boolean isItRaining) {
        try {
            InputStream input = PhysicsEngine.class.getResourceAsStream("/FrictionCoefficient.csv");
            Reader in = new InputStreamReader(input);
            CSVParser csvParser = new CSVParser(in, CSVFormat.DEFAULT);

            switch (streetPavement) {
                case QUALITY: // Asphalt
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(1).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(1).get(2));
                    }

                case STONE:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(2).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(2).get(2));
                    }

                case PAVED:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(3).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(3).get(2));
                    }

                case DIRT:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(4).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(4).get(2));
                    }

                case UNPAVED:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(5).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(5).get(2));
                    }

                case GRASS:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(6).get(1));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(6).get(2));
                    }

                default:
                    return 1;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        return 1;
    }

    public static double calcRollingResistance(Vec3 v, double pressure, double forceRoadFrictionBackFrontNorm) {
        double defaultValue = 0.005 + (1 / pressure) * (0.01 + 0.0095 * (forceRoadFrictionBackFrontNorm * 3.6 / 100) * (forceRoadFrictionBackFrontNorm * 3.6 / 100));
        try {
            StreetPavements streetPavement = World.getInstance().getSurfaceType(v);
            boolean isItRaining = World.getInstance().isItRaining();
            InputStream input = PhysicsEngine.class.getResourceAsStream("/FrictionCoefficient.csv");
            Reader in = new InputStreamReader(input);
            CSVParser csvParser = new CSVParser(in, CSVFormat.DEFAULT);


            switch (streetPavement) {
                case QUALITY:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(1).get(3));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(1).get(4));
                    }
                case STONE:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(2).get(3));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(2).get(4));
                    }
                case PAVED:// Asphalt
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(3).get(3));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(3).get(4));
                    }
                case DIRT:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(4).get(3));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(4).get(4));
                    }
                case UNPAVED:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(5).get(3));
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(5).get(4));
                    }
                case GRASS:
                    if (isItRaining) {
                        return Double.parseDouble(csvParser.getRecords().get(6).get(3)); // provisional value
                    } else {
                        return Double.parseDouble(csvParser.getRecords().get(6).get(4));
                    }

                default:
                    return defaultValue;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        return defaultValue;
    }
}
