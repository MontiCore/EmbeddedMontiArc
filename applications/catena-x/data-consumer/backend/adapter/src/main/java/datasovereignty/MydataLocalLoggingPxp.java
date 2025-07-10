package datasovereignty;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import lombok.extern.slf4j.Slf4j;

import java.time.LocalDateTime;

@PxpService(componentName = "logging-local-pxp")
@Slf4j
public class MydataLocalLoggingPxp {

	@ActionDescription(methodName = "log-local")
	public boolean log(@ActionParameterDescription(name = "id", mandatory = true) final String datasetId) {
		log.info("Dataset {} accessed at {}", datasetId, LocalDateTime.now());

		return true;
	}
}
