package usecases;

import dto.DatasetView;
import entity.Dataset;
import lombok.AllArgsConstructor;
import ports.DatasetPersistencePort;
import ports.InsuranceFeeCalculatorPort;
import ports.PolicyEnforcementPort;
import queries.GetDatasetQuery;

@AllArgsConstructor
public class GetDatasetViewUseCase implements QueryHandler<GetDatasetQuery, DatasetView> {

	private final DatasetPersistencePort datasetPersistencePort;
	private final InsuranceFeeCalculatorPort feeCalculatorPort;
	private final PolicyEnforcementPort<Dataset> enforcementPort;

	public DatasetView handle(GetDatasetQuery query) {
		Dataset dataset = enforcementPort.enforcePolicy(datasetPersistencePort.findById(query.getDatasetId()));

		return new DatasetView(
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
