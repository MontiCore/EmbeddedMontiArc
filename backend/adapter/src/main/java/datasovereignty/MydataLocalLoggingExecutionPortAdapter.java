package datasovereignty;

import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import ports.LocalLoggingExecutionPort;

import java.time.LocalDateTime;

@Component
@Slf4j
public class MydataLocalLoggingExecutionPortAdapter implements LocalLoggingExecutionPort {

	@Override
	@ActionDescription(methodName = "log-local")
	public boolean log(@ActionParameterDescription(name = "id", mandatory = true) final String datasetId) {
		log.info("Dataset {} accessed at {}", datasetId, LocalDateTime.now());

		return true;
	}

	@Override
	public String getName() {
		return "logging-local-pxp";
	}
}
