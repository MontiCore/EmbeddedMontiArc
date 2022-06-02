package datasovereignty;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import entity.Dataset;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import ports.DatasetPersistencePort;

import java.time.LocalDate;

@PxpService(componentName = "expiration-check-pxp")
@AllArgsConstructor
@Slf4j
public class MydataExpirationCheckExecutionPortAdapter {

	private DatasetPersistencePort persistencePort;

	@ActionDescription(methodName = "check-expiration")
	public boolean removeExpiredDatasets() {
		for (Dataset dataset : persistencePort.findAll()) {
			LocalDate expiresOn = dataset.getMetadata().getPolicy().getExpiresOn();
			if (expiresOn != null && expiresOn.isBefore(LocalDate.now())) {
				persistencePort.deleteById(dataset.getId());

				log.info("Dataset {} was deleted because it was expired", dataset.getId());
			}
		}

		return true;
	}
}
