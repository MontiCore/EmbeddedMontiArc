package datasovereignty;

import de.fraunhofer.iese.mydata.registry.ActionDescription;
import entity.Dataset;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import ports.DatasetPersistencePort;
import ports.ExpirationCheckExecutionPoint;

import java.time.LocalDate;

@Component
@AllArgsConstructor
@Slf4j
public class ExpirationCheckExecutionPointAdapter implements ExpirationCheckExecutionPoint {

	private DatasetPersistencePort persistencePort;

	@Override
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

	@Override
	public String getName() {
		return "expiration-check-pxp";
	}
}
