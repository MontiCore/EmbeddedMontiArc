package usecase;

import entity.Dataset;
import entity.Offer;
import lombok.AllArgsConstructor;
import port.DatasetPersistencePort;
import port.OfferPersistencePort;
import port.PolicyManagementPort;

import java.time.LocalDateTime;
import java.util.UUID;

@AllArgsConstructor
public class BuyOfferUseCase {

	private OfferPersistencePort offerPersistencePort;
	private DatasetPersistencePort datasetPersistencePort;
	private PolicyManagementPort policyManagementPort;

	public void buyOffer(UUID offerId) {
		Offer offer = offerPersistencePort.findBy(offerId);
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
