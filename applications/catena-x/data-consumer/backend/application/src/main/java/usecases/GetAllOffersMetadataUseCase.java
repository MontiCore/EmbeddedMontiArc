package usecases;

import entity.Metadata;
import entity.Offer;
import lombok.AllArgsConstructor;
import ports.OfferPersistencePort;
import queries.GetAllOffersMetadataQuery;

import java.util.HashMap;
import java.util.Map;
import java.util.UUID;

@AllArgsConstructor
public class GetAllOffersMetadataUseCase implements QueryHandler<GetAllOffersMetadataQuery, Map<UUID, Metadata>> {

	private final OfferPersistencePort offerPersistencePort;

	@Override
	public Map<UUID, Metadata> handle(GetAllOffersMetadataQuery query) {
		Map<UUID, Metadata> map = new HashMap<>();
		Iterable<Offer> offers = offerPersistencePort.findAll();
		offers.forEach(offer -> map.put(offer.getId(), offer.getMetadata()));

		return map;
	}
}
