package usecases;

import dto.DatasetViewDto;
import entity.Dataset;
import lombok.AllArgsConstructor;
import ports.DatasetPersistencePort;
import ports.InsuranceFeeCalculatorPort;
import ports.PolicyEnforcementPort;
import queries.GetDatasetQuery;

@AllArgsConstructor
public class GetDatasetViewUseCase implements QueryHandler<GetDatasetQuery, DatasetViewDto> {

	private final DatasetPersistencePort datasetPersistencePort;
	private final InsuranceFeeCalculatorPort feeCalculatorPort;
	private final PolicyEnforcementPort<Dataset> enforcementPort;

	public DatasetViewDto handle(GetDatasetQuery query) {
		Dataset dataset = enforcementPort.enforcePolicy(datasetPersistencePort.findById(query.getDatasetId()));

		return new DatasetViewDto(
				dataset.getId(),
				dataset.getMetadata(),
				dataset.getData(),
				dataset.getBoughtAt(),
				dataset.getNumberOfTrucks(),
				dataset.getRestingTrucks(),
				dataset.getDrivingTrucks(),
				dataset.getFuelConsumption(),
				feeCalculatorPort.calculateFee(dataset)
		);
	}
}
