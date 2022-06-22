package usecases;

import commands.DeleteDatasetCommand;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import ports.DatasetPersistencePort;

@AllArgsConstructor
@Slf4j
public class DeleteDatasetUseCase implements CommandHandler<DeleteDatasetCommand> {

	private DatasetPersistencePort datasetPersistencePort;

	@Override
	public void handle(DeleteDatasetCommand command) {
		datasetPersistencePort.deleteById(command.getDatasetId());

		log.info("Successfully deleted dataset {}", command.getDatasetId().toString());
	}
}
