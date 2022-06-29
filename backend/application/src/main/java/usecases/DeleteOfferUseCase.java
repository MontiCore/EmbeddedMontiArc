package usecases;

import commands.DeleteOfferCommand;
import entity.Offer;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import ports.DatasetPersistencePort;
import ports.OfferPersistencePort;

@AllArgsConstructor
@Slf4j
public class DeleteOfferUseCase implements CommandHandler<DeleteOfferCommand> {

	private DatasetPersistencePort datasetPersistencePort;
	private OfferPersistencePort offerPersistencePort;


	@Override
	public void handle(DeleteOfferCommand command) {
		Offer offer = offerPersistencePort.findById(command.getOfferId());
		if (offer != null) {
			offerPersistencePort.deleteDatasetDataRowOfferById(command.getOfferId());
		}
		datasetPersistencePort.deleteByOfferId(command.getOfferId());

		log.info("Successfully deleted dataset {}", command.getOfferId().toString());
	}
}
