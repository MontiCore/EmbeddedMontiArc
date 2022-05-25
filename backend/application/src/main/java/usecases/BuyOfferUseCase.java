package usecases;

import commands.BuyOfferCommand;
import entity.Dataset;
import entity.Offer;
import lombok.AllArgsConstructor;
import ports.DatasetPersistencePort;
import ports.OfferPersistencePort;
import ports.PolicyManagementPort;

import java.time.LocalDateTime;
import java.util.UUID;

@AllArgsConstructor
public class BuyOfferUseCase implements CommandHandler<BuyOfferCommand> {

	private OfferPersistencePort offerPersistencePort;
	private DatasetPersistencePort datasetPersistencePort;
	private PolicyManagementPort policyManagementPort;

	@Override
	public void handle(BuyOfferCommand command) {
		Offer offer = offerPersistencePort.findBy(command.getOfferId());
		policyManagementPort.deployPolicy(offer.getPolicy());
		Dataset dataset = mapOfferToDataset(offer);
		datasetPersistencePort.save(dataset);
	}

	private Dataset mapOfferToDataset(Offer offer) {
		return new Dataset(
				UUID.randomUUID(),
				offer.getTitle(),
				offer.getProvider(),
				offer.getDescription(),
				offer.getPrice(),
				offer.getPolicy(),
				LocalDateTime.now(),
				offer.getExpiresOn(),
				offer.getData(),
				offer.getLoggingUrl()
		);
	}
}
