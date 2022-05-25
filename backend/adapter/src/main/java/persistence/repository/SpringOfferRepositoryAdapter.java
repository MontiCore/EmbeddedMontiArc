package persistence.repository;

import entity.Offer;
import lombok.AllArgsConstructor;
import persistence.entity.OfferEntity;
import persistence.mappers.JacksonOfferMapper;
import ports.OfferPersistencePort;

import java.util.UUID;

@AllArgsConstructor
public class SpringOfferRepositoryAdapter implements OfferPersistencePort {

	private final JacksonOfferMapper mapper;
	private final SpringOfferRepository repository;

	@Override
	public void save(Offer offer) {
		OfferEntity offerEntity = mapper.mapToPersistenceEntity(offer);
		offerEntity.getData().forEach(dataRowEntity -> dataRowEntity.setOffer(offerEntity));

		repository.save(offerEntity);
	}

	@Override
	public Iterable<Offer> findAll() {
		return null;
	}

	@Override
	public Offer findBy(UUID offerId) {
		return null;
	}
}
