package datasovereignty;

import commands.RemoveExpiredDatasetsCommand;
import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import ports.DatasetPersistencePort;
import usecases.RemoveExpiredDatasetsUseCase;

@PxpService(componentName = "expiration-check-pxp")
@AllArgsConstructor
@Slf4j
public class MydataExpirationCheckExecutionPortAdapter {

	private DatasetPersistencePort persistencePort;
	private RemoveExpiredDatasetsUseCase removeExpiredDatasetsUseCase;

	@ActionDescription(methodName = "check-expiration")
	public boolean removeExpiredDatasets() {
		removeExpiredDatasetsUseCase.handle(new RemoveExpiredDatasetsCommand());

		return true;
	}
}
