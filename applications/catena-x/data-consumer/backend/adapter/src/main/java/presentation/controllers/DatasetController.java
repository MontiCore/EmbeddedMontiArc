package presentation.controllers;

import commands.DeleteDatasetCommand;
import dto.DatasetViewDto;
import entity.Metadata;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.web.bind.annotation.*;
import queries.GetAllDatasetMedataQuery;
import queries.GetDatasetQuery;
import usecases.DeleteDatasetUseCase;
import usecases.GetAllDatasetMetadataUseCase;
import usecases.GetDatasetViewUseCase;

import java.util.Map;
import java.util.UUID;

@RestController
@RequestMapping("/datasets")
@AllArgsConstructor
@CrossOrigin(origins = "*")
@Slf4j
public class DatasetController {

	private DeleteDatasetUseCase deleteDatasetUseCase;
	private GetDatasetViewUseCase getDatasetViewUseCase;
	private GetAllDatasetMetadataUseCase getAllDatasetMetadataUseCase;

	@GetMapping
	public Map<UUID, Metadata> getAllDatasets() {
		return getAllDatasetMetadataUseCase.handle(new GetAllDatasetMedataQuery());
	}

	@GetMapping("/{datasetId}")
	public DatasetViewDto getDataset(@PathVariable UUID datasetId) {
		return getDatasetViewUseCase.handle(new GetDatasetQuery(datasetId));
	}

	@DeleteMapping("/{datasetId}")
	public void deleteDataset(@PathVariable UUID datasetId) {
		deleteDatasetUseCase.handle(new DeleteDatasetCommand(datasetId));
	}
}
