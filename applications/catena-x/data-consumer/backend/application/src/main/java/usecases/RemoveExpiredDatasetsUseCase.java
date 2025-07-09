package usecases;

import commands.RemoveExpiredDatasetsCommand;
import entity.Dataset;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import ports.DatasetPersistencePort;

import java.time.LocalDate;


@AllArgsConstructor
@Slf4j
public class RemoveExpiredDatasetsUseCase implements CommandHandler<RemoveExpiredDatasetsCommand> {

	private final DatasetPersistencePort persistencePort;

	public void handle(RemoveExpiredDatasetsCommand command) {
		for (Dataset dataset : persistencePort.findAll()) {
			LocalDate expiresOn = dataset.getMetadata().getPolicy().getExpiresOn();
			if (expiresOn != null && expiresOn.isBefore(LocalDate.now())) {
				persistencePort.deleteById(dataset.getId());

				log.info("Dataset {} was deleted because it was expired", dataset.getId());
			}
		}
	}
}
