/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.commons.utils.json.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;
import de.rwth.montisim.simulation.eesimulator.exceptions.*;

public class VehicleBuilder {

    public static VehicleBuilder fromConfig(BuildContext context, VehicleProperties config) {
        VehicleBuilder builder = new VehicleBuilder(context);
        builder.config = Optional.of(config);
        return builder;
    }

    public static VehicleBuilder fromJsonConfig(BuildContext context, File configFile) {
        VehicleBuilder builder = new VehicleBuilder(context);
        builder.file = Optional.of(configFile);
        builder.fromJson = true;
        builder.isState = false;
        return builder;
    }

    public static VehicleBuilder fromJsonConfig(BuildContext context, String configData) {
        VehicleBuilder builder = new VehicleBuilder(context);
        builder.dataString = Optional.of(configData);
        builder.fromJson = true;
        builder.isState = false;
        return builder;
    }

    public static VehicleBuilder fromJsonState(BuildContext context, File configFile) {
        VehicleBuilder builder = new VehicleBuilder(context);
        builder.file = Optional.of(configFile);
        builder.fromJson = true;
        builder.isState = true;
        return builder;
    }

    public static VehicleBuilder fromJsonState(BuildContext context, String configData) {
        VehicleBuilder builder = new VehicleBuilder(context);
        builder.dataString = Optional.of(configData);
        builder.fromJson = true;
        builder.isState = true;
        return builder;
    }

    final BuildContext context;
    // final Vehicle target;
    boolean fromJson = false;
    boolean isState = false;
    Optional<VehicleProperties> config = Optional.empty();
    Optional<File> file = Optional.empty();
    Optional<String> dataString = Optional.empty();

    private VehicleBuilder(BuildContext context) {
        this.context = context;
    }


    public Vehicle build() throws SerializationException, EEMessageTypeException, EESetupException,
            EEMissingComponentException {
        if (fromJson) {
            JsonTraverser j = new JsonTraverser();
            try {
                if (file.isPresent()) {
                    j.init(file.get());
                } else {
                    j.init(dataString.get());
                }
            } catch (ParsingException | IOException e) {
                throw new SerializationException(e);
            }

            if (isState) {
                ObjectIterable it = j.streamObject();

                if (!it.iterator().hasNext())
                    j.expected(Vehicle.K_CONFIG);
                Entry e = it.iterator().next();
                if (!e.key.equals(Vehicle.K_CONFIG))
                    j.expected(Vehicle.K_CONFIG);
                config = Optional.of(Json.instantiateFromJson(j, VehicleProperties.class, null));
                Vehicle target = buildFromConfig();

                if (!it.iterator().hasNext())
                    j.expected(Vehicle.K_STATE);
                e = it.iterator().next();
                if (!e.key.equals(Vehicle.K_STATE))
                    j.expected(Vehicle.K_STATE);
                Json.fromJson(j, target, null);
                return target;
            } else {
                config = Optional.of(Json.instantiateFromJson(j, VehicleProperties.class, null));
                return buildFromConfig();
            }
        } else {
            return buildFromConfig();
        }
    }

    private Vehicle buildFromConfig() throws EEMessageTypeException, EESetupException, EEMissingComponentException {
        return config.get().build(context);
    }

}