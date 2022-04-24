package de.thesis.consumer.backend.persistence.repository;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.model.Policy;
import de.thesis.consumer.backend.domain.repository.PolicyRepository;
import de.thesis.consumer.backend.persistence.entity.PolicyEntity;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Repository;

@Repository
@AllArgsConstructor
public class PolicyRepositoryImpl implements PolicyRepository {

	// TODO kann man das ganze mapping mit einer Generics-Klasse machen?
	// TODO Optionalhandling fixen
	private final SpringDataPolicyCrudRepository repository;
	private final ObjectMapper mapper;

	@Override
	public void save(Policy policy) {
		PolicyEntity entity = mapper.convertValue(policy, PolicyEntity.class);
		repository.save(entity);
	}

	@Override
	public Policy findById(String id) {
		PolicyEntity entity = repository.findById(id).orElseThrow(null);
		return mapper.convertValue(entity, Policy.class);
	}
}
