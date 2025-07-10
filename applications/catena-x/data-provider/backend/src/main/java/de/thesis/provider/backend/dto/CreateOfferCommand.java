package de.thesis.provider.backend.dto;


import de.thesis.provider.backend.csv.DataRow;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.NoArgsConstructor;
import lombok.Value;

import java.util.List;

@Value
@NoArgsConstructor(force = true, access = AccessLevel.PRIVATE)
@AllArgsConstructor
public class CreateOfferCommand {
	Metadata metadata;
	List<DataRow> data;
}
