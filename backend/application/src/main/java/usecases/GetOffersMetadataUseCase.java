package usecases;

import entity.Metadata;
import entity.Offer;
import lombok.AllArgsConstructor;
import ports.OfferPersistencePort;
import queries.GetOffersMetadataQuery;

import java.util.HashMap;
import java.util.Map;
import java.util.UUID;

@AllArgsConstructor
public class GetOffersMetadataUseCase implements QueryHandler<GetOffersMetadataQuery, Map<UUID, Metadata>> {

	private final OfferPersistencePort offerPersistencePort;

	@Override
	public Map<UUID, Metadata> handle(GetOffersMetadataQuery query) {
		Map<UUID, Metadata> map = new HashMap<>();
		Iterable<Offer> offers;

		if (query.isOnlyBought()) {
			offers = offerPersistencePort.findAllBought();
		} else {
			offers = offerPersistencePort.findAll();
		}

		offers.forEach(offer -> map.put(offer.getId(), offer.getMetadata()));

		return map;
	}
}
