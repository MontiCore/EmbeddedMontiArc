package datasovereignty;

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
	public boolean removeExpiredDatasets() {
		for (Dataset dataset : persistencePort.findAll()) {
			if (dataset.getMetadata().getExpiresOn() != null &&
					dataset.getMetadata().getExpiresOn().isBefore(LocalDate.now())) {
				log.info("Dataset {} was removed because it was expired", dataset.getId());
				persistencePort.deleteById(dataset.getId());
			}
		}

		return true;
	}

	@Override
	public String getName() {
		return "expiration-check-pxp";
	}
}
