package usecases;

import commands.DeleteDatasetCommand;
import commands.DeleteOfferCommand;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import ports.DatasetPersistencePort;
import ports.OfferPersistencePort;

@AllArgsConstructor
@Slf4j
public class DeleteOfferUseCase implements CommandHandler<DeleteOfferCommand> {

	private OfferPersistencePort offerPersistencePort;


	@Override
	public void handle(DeleteOfferCommand command) {
		offerPersistencePort.deleteDatasetDataRowOfferById(command.getOfferId());

		log.info("Successfully deleted dataset {}", command.getOfferId().toString());
	}
}
