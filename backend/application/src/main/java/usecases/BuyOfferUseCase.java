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
		policyManagementPort.deployPolicy(offer);
		Dataset dataset = createDatasetFromOffer(offer);
		dataset.setBoughtAt(LocalDateTime.now());
		datasetPersistencePort.save(dataset);

		return dataset;
	}

	private Dataset createDatasetFromOffer(Offer offer) {
		Dataset dataset = new Dataset();
		dataset.setId(offer.getId());
		dataset.setMetadata(offer.getMetadata());
		dataset.setData(offer.getData());

		return dataset;
	}
}
