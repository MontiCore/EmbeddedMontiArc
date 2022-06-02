package usecases;

import commands.BuyOfferCommand;
import entity.Dataset;
import entity.Offer;
import lombok.AllArgsConstructor;
import ports.DatasetPersistencePort;
import ports.OfferPersistencePort;
import ports.DsManagementPort;

import java.time.LocalDateTime;

@AllArgsConstructor
public class BuyOfferUseCase implements CommandHandler<BuyOfferCommand, Dataset> {

	private DatasetPersistencePort datasetPersistencePort;
	private OfferPersistencePort offerPersistencePort;
	private DsManagementPort dsManagementPort;

	@Override
	public Dataset handle(BuyOfferCommand command) {
		Offer offer = offerPersistencePort.findBy(command.getOfferId());
		// Policy policy = offer.getMetadata().getPolicy();
		dsManagementPort.deployPolicy(offer);
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
