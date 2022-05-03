package de.thesis.provider.backend.service;

import de.thesis.provider.backend.DatasetCreateCommand;
import de.thesis.provider.backend.InsuranceClient;
import de.thesis.provider.backend.Offer;
import de.thesis.provider.backend.Policy;
import de.thesis.provider.backend.csv.CsvReader;
import de.thesis.provider.backend.csv.DataRow;
import de.thesis.provider.backend.dto.CreatePolicyDto;
import de.thesis.provider.backend.policy.PolicyService;
import freemarker.template.TemplateException;
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
	private final PolicyService policyService;

	public void offerDataset(DatasetCreateCommand command) throws IOException, TemplateException {
		UUID id = UUID.randomUUID();
		Offer offer = new Offer();
		offer.setId(id);
		offer.setTitle(command.getMetaData().getTitle());
		offer.setDescription(command.getMetaData().getDescription());
		offer.setPrice(command.getMetaData().getPrice());
		offer.setExpiresOn(command.getPolicy().getExpiresOn());
		CreatePolicyDto policy = command.getPolicy();
		policy.setId(id);
		policy.setEvent("data-access");
		offer.setPolicy(new Policy("urn:policy:rwth-student-solution:" + id, policyService.getPolicy(policy)));
		offer.setData(getDataRows(command));

		client.offerDataset(offer);
	}

	private List<DataRow> getDataRows(DatasetCreateCommand command) throws IOException {
		CsvReader reader = new CsvReader();
		List<DataRow> truckData = reader.getCsvData();
		String[] rows = command.getRows().split(";");
		List<DataRow> datasetData = new ArrayList<>();
		for(String interval : rows) {
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
