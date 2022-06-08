package entity;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;
import java.util.UUID;

@Data
@AllArgsConstructor
@NoArgsConstructor
public class Dataset {
	private UUID id;
	private Offer offer;
	private Metadata metadata;
	private List<DataRow> data;
	private LocalDateTime boughtAt;
}
