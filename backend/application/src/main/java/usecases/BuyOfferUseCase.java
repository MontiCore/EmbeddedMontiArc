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
public class BuyOfferUseCase implements CommandHandler<BuyOfferCommand, Dataset> {

	private DatasetPersistencePort datasetPersistencePort;
	private OfferPersistencePort offerPersistencePort;
	private PolicyManagementPort policyManagementPort;

	@Override
	public Dataset handle(BuyOfferCommand command) {
		Offer offer = offerPersistencePort.findBy(command.getOfferId());
		policyManagementPort.deployPolicy(offer.getPolicy());
		Dataset dataset = createDatasetFromOffer(offer);
		dataset.setBoughtAt(LocalDateTime.now());
		datasetPersistencePort.save(dataset);

		return dataset;
	}

	private Dataset createDatasetFromOffer(Offer offer) {
		Dataset dataset = new Dataset();
		dataset.setId(UUID.randomUUID());
		dataset.setTitle(offer.getTitle());
		dataset.setProvider(offer.getProvider());
		dataset.setDescription(offer.getDescription());
		dataset.setPrice(offer.getPrice());
		dataset.setPolicy(offer.getPolicy());
		dataset.setExpiresOn(offer.getExpiresOn());
		dataset.setData(offer.getData());
		dataset.setLoggingUrl(offer.getLoggingUrl());

		return dataset;
	}
}
