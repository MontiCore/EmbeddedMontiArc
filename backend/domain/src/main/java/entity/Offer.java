package entity;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Value;

import java.time.LocalDate;
import java.util.List;
import java.util.UUID;

@Data
@AllArgsConstructor
public class Offer {
	private UUID id;
	private Metadata metadata;
	private List<DataRow> data;
}
