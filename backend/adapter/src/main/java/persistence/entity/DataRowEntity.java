package persistence.entity;

import com.fasterxml.jackson.annotation.JsonIgnore;
import lombok.Getter;
import lombok.Setter;

import javax.persistence.*;
import java.time.LocalDateTime;

@Getter
@Setter
@Entity
@Table(name = "data_row")
public class DataRowEntity {
	@Id
	private int id;
	private String dayID;
	private double longitude;
	private double latitude;
	private LocalDateTime gpsTime;
	private double heading;
	private int speed;
	private int odometer;
	private int totalFuelUsed;
	private LocalDateTime timestamp;

	@ManyToOne
	@JoinColumn(name = "offer")
	@JsonIgnore
	private OfferEntity offer;

	@ManyToOne
	@JoinColumn(name = "dataset")
	@JsonIgnore
	private DatasetEntity dataset;

}
