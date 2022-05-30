package de.thesis.provider.backend.service;

import de.thesis.provider.backend.CreateDatasetCommand;
import de.thesis.provider.backend.Dataset;
import de.thesis.provider.backend.InsuranceClient;
import de.thesis.provider.backend.Metadata;
import de.thesis.provider.backend.csv.CsvReader;
import de.thesis.provider.backend.csv.DataRow;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

@Component
@AllArgsConstructor
public class DatasetService {

	private final InsuranceClient client;

	public void offerDataset(CreateDatasetCommand command) throws IOException {
		Dataset dataset = new Dataset();
		dataset.setId(UUID.randomUUID());
		dataset.setMetadata(command.getMetadata());
		dataset.setData(getDataRows(command));

		client.offerDataset(dataset);
	}

	public List<Metadata> getDatasets() {
		return client.getOffers();
	}

	private List<DataRow> getDataRows(CreateDatasetCommand command) throws IOException {
		CsvReader reader = new CsvReader();
		List<DataRow> truckData = reader.getCsvData();
		String[] rows = command.getRows().split(";");
		List<DataRow> datasetData = new ArrayList<>();
		for (String interval : rows) {
			if (interval.contains("-")) {
				int start = Integer.parseInt(interval.substring(0, interval.indexOf("-")));
				int end = Integer.parseInt(interval.substring(interval.indexOf("-") + 1));

				datasetData.addAll(truckData.subList(start, end));
			} else {
				datasetData.add(truckData.get(Integer.parseInt(interval)));
			}
		}

		return datasetData;
	}
}
