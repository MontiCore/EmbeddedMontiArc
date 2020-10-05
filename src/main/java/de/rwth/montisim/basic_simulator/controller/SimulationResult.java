/**
 * (c) https://github.com/MontiCore/monticore
 */
// package de.rwth.montisim.basic_simulator.controller;

// import java.io.BufferedOutputStream;
// import java.io.File;
// import java.io.FileOutputStream;
// import java.text.SimpleDateFormat;
// import java.util.ArrayList;
// import java.util.Date;
// import java.util.HashMap;
// import java.util.LinkedList;
// import java.util.List;
// import java.util.Map;

// import javax.json.Json;
// import javax.json.JsonArray;
// import javax.json.JsonArrayBuilder;
// import javax.json.JsonNumber;
// import javax.json.JsonObject;
// import javax.json.JsonObjectBuilder;
// import javax.json.JsonValue;
// import javax.json.JsonWriter;
// import javax.json.JsonValue.ValueType;

// import de.rwth.montisim.basic_simulator.filesystem.FileSystem;

// public class SimulationResult {

//     public enum Entries {
//         TYPE("type", true, "result"),
//         FRAME_INFO("frame_info", true, "\"String\""),
//         MAP("map", true, "Map name."),
//         VEHICLES("vehicles", true, "Dictionary of <UUID:Car object>"),
//         CONFIG("config", false, "Scenario configuration");

//         public String key_name;
//         public boolean required;
//         public String format;
//         Entries(String key_name, boolean required, String format){
//             this.key_name = key_name;
//             this.required = required;
//             this.format = format;
//         }
//     }

//     public enum CarEntries {
//         START_TIME("start_time", true, "Simulation start time."),
//         NAME("name", false, "Car name"),
//         CONFIG("config", false, "Car config"),
//         TRAJ_X("planned_trajectory_x", true, ""),
//         TRAJ_Y("planned_trajectory_y", true, ""),
//         POS_X("position_x", false, ""),
//         POS_Y("position_y", false, ""),
//         ORIENTATION("orientation", false, ""),
//         GAS("gas", false, ""),
//         BRAKES("brakes", false, ""),
//         STEERING("steering", false, "");

//         public String key_name;
//         public boolean required;
//         public String format;
//         CarEntries(String key_name, boolean required, String format){
//             this.key_name = key_name;
//             this.required = required;
//             this.format = format;
//         }

//         public static boolean[] getBoolArray(){
//             return new boolean[values().length];
//         }
//     }

//     public class CarFrame {
//         public double x, y, orientation, brakes, gas, steering;

//         public CarFrame(double x, double y, double orientation, double brakes, double gas, double steering) {
//             this.x = x;
//             this.y = y;
//             this.orientation = orientation;
//             this.brakes = brakes;
//             this.gas = gas;
//             this.steering = steering;
//         }
//     }

//     public class SimulationFrame {
//         public HashMap<Long, CarFrame> car_frames;
//         long time;
//         public SimulationFrame(long time){
//             this.time = time;
//             car_frames = new HashMap<>();
//         }
//     }

//     public class CarInfo {
//         public boolean encountered;
//         public boolean arrived;
//         public String name;
//         public JsonObject config;
//         public long start_time;
//         public Point2D target;
//         public Point2D[] planned_trajectory;
//         public CarInfo(String name, JsonObject config, VehicleBuilder.VehicleTrajectory trajectory) {
//             this.name = name;
//             this.config = config;
//             this.planned_trajectory = new Point2D[trajectory.trajectory.length];
//             int i = 0;
//             for (Point2D p : trajectory.trajectory){
//                 planned_trajectory[i] = new Point2D(p.getX(), p.getY());
//                 i++;
//             }
//             this.target = trajectory.target;
//             this.encountered = false;
//             this.arrived = false;
//             this.start_time = 0;
//         }
//         public CarInfo(JsonObject from_file, List<SimulationFrame> frames, long id) throws Exception{
//             for (CarEntries entry : CarEntries.values()){
//                 if (entry.required){
//                     if(!from_file.containsKey(entry.key_name)){
//                         throw new Exception("Car result missing entry: " + entry.key_name + ": " + entry.format);
//                     }
//                 }
//             }
//             JsonNumber st = from_file.getJsonNumber(CarEntries.START_TIME.key_name);
//             if (st == null)
//                 throw new Exception("Invalid Car start_time entry.");
//             this.start_time = st.longValueExact();
//             JsonArray pl_x = from_file.getJsonArray(CarEntries.TRAJ_X.key_name);
//             JsonArray pl_y = from_file.getJsonArray(CarEntries.TRAJ_Y.key_name);
//             if (pl_x == null || pl_y == null || pl_x.size() != pl_y.size() || pl_y.size() == 0)
//                 throw new Exception("Invalid Car planned trajectory.");
//             this.planned_trajectory = new Point2D[pl_x.size()];
//             for (int i = 0; i < pl_x.size(); i++){
//                 JsonValue v_x = pl_x.get(i);
//                 JsonValue v_y = pl_y.get(i);
//                 if (v_x.getValueType() != ValueType.NUMBER || v_y.getValueType() != ValueType.NUMBER)
//                     throw new Exception("Invalid Car planned trajectory entry.");
//                 planned_trajectory[i] = new Point2D(((JsonNumber)v_x).doubleValue(), ((JsonNumber)v_y).doubleValue());
//             }

//             //TODO CONFIG

//             JsonArray a_x = from_file.getJsonArray(CarEntries.POS_X.key_name);
//             JsonArray a_y = from_file.getJsonArray(CarEntries.POS_Y.key_name);
//             JsonArray a_orientation = from_file.getJsonArray(CarEntries.ORIENTATION.key_name);
//             JsonArray a_gas = from_file.getJsonArray(CarEntries.GAS.key_name);
//             JsonArray a_brakes = from_file.getJsonArray(CarEntries.BRAKES.key_name);
//             JsonArray a_steering = from_file.getJsonArray(CarEntries.STEERING.key_name);

//             int i = 0;
//             for (SimulationFrame frame : frames){
//                 if (frame.time >= start_time){
//                     boolean one = false;
//                     double x =0, y=0,orientation=0, brakes=0, gas=0, steering=0;
//                     if (a_x != null && i < a_x.size()){
//                         JsonNumber n = a_x.getJsonNumber(i);
//                         if (n != null){
//                             x = n.doubleValue();
//                             one = true;
//                         }
//                     }
//                     if (a_y != null && i < a_y.size()){
//                         JsonNumber n = a_y.getJsonNumber(i);
//                         if (n != null){
//                             y = n.doubleValue();
//                             one = true;
//                         }
//                     }
//                     if (a_orientation != null && i < a_orientation.size()){
//                         JsonNumber n = a_orientation.getJsonNumber(i);
//                         if (n != null){
//                             orientation = n.doubleValue();
//                             one = true;
//                         }
//                     }
//                     if (a_gas != null && i < a_gas.size()){
//                         JsonNumber n = a_gas.getJsonNumber(i);
//                         if (n != null){
//                             gas = n.doubleValue();
//                             one = true;
//                         }
//                     }
//                     if (a_brakes != null && i < a_brakes.size()){
//                         JsonNumber n = a_brakes.getJsonNumber(i);
//                         if (n != null){
//                             brakes = n.doubleValue();
//                             one = true;
//                         }
//                     }
//                     if (a_steering != null && i < a_steering.size()){
//                         JsonNumber n = a_steering.getJsonNumber(i);
//                         if (n != null){
//                             steering = n.doubleValue();
//                             one = true;
//                         }
//                     }
//                     if (!one) break;
//                     CarFrame car_frame = new CarFrame(x, y, orientation, brakes, gas, steering);
//                     frame.car_frames.put(id, car_frame);
//                     i++;
//                 }
//             }
//         }
//     }




//     FileSystem fileSystem;
//     public List<SimulationFrame> frames;
//     public HashMap<Long, CarInfo> cars;
//     public SimulationFrame current_frame;
//     int car_count;
//     public String name;
//     public String map;
//     public int simulation_frequency;
//     public long start_time; //Real time timestamp of simulation start
//     boolean export_timestamp;

//     public Point2D min_coords;
//     public Point2D max_coords;
//     public Point2D coord_span;

//     public SimulationResult(FileSystem fileSystem){
//         this.fileSystem = fileSystem;
//         min_coords = new Point2D(0,0);
//         max_coords = new Point2D(1,1);
//         coord_span = new Point2D(1,1);
//     }

    
//     public void init_for_simulation(String name, String map, boolean export_timestamp, int simulation_frequency){
//         this.name = name;
//         this.map = map;
//         this.export_timestamp = export_timestamp;
//         this.simulation_frequency = simulation_frequency;
//         frames = new LinkedList<SimulationFrame>();
//         cars = new HashMap<Long, CarInfo>();
//         car_count = 0;
//     }

//     public void register_car(long id, String name, JsonObject config, VehicleBuilder.VehicleTrajectory trajectory){
//         cars.put(id, new CarInfo(name, config, trajectory));
//         car_count++;
//     }

//     public void simulation_start(){
//         compute_span();
//         start_time = System.currentTimeMillis();
//     }

//     private void compute_span(){
//         double min_x = Double.POSITIVE_INFINITY, min_y = Double.POSITIVE_INFINITY;
//         double max_x = Double.NEGATIVE_INFINITY, max_y = Double.NEGATIVE_INFINITY;
//         for (Map.Entry<Long, CarInfo> c : cars.entrySet()){
//             for (Point2D p : c.getValue().planned_trajectory){
//                 double x = p.getX();
//                 if (x < min_x) min_x = x;
//                 if (x > max_x) max_x = x;
//                 double y = p.getY();
//                 if (y < min_y) min_y = y;
//                 if (y > max_y) max_y = y;
//             }
//         }
//         Point2D border = new Point2D(5,5);
//         min_coords = new Point2D(min_x, min_y).subtract(border);
//         max_coords = new Point2D(max_x, max_y).add(border);
//         coord_span = max_coords.subtract(min_coords);
//     }

//     public void setup_next_frame(long totalTime){
//         current_frame = new SimulationFrame(totalTime);
//         frames.add(current_frame);
//     }

//     //returns true if the simulation is completed
//     public boolean add_car_frame(PhysicalVehicle vehicle){
//         long id = vehicle.getId();
//         CarInfo info = cars.get(id);

//         if (!info.encountered){
//             info.encountered = true;
//             info.start_time = current_frame.time;
//         }

//         double[] position = vehicle.getPosition().toArray();
//         double x = position[0];
//         double y = position[1];
//         double dist = info.target.distance(new Point2D(x,y));
//         System.out.println("\tPosition of "+ info.name + ": [" +x + ", " + y + "] (Dist: " + dist + ")");

//         current_frame.car_frames.put(id, new CarFrame(x, y, 0,0,0,0));
//         //Temporary hack to stop simulation when vehicle attained its target
//         if (!info.arrived){
//             if (dist < 1){
//                 System.out.println("\tCar at target");
//                 info.arrived = true;
//                 car_count--;

//                 if (car_count == 0){
//                     return true;
//                 }
//             }
//         }
//         return false;
//     }
    

//     public void export(){
//         String result_name = name;
//         if (export_timestamp){
//             SimpleDateFormat sdf = new SimpleDateFormat("yyyy.MM.dd-HH.mm.ss");
//             String date = sdf.format(new Date(start_time));
//             result_name += "-" + date;
//         }

//         JsonArrayBuilder frame_times = Json.createArrayBuilder();
//         for (SimulationFrame frame : frames){
//             frame_times.add(frame.time);
//         }

//         JsonObject time_info = Json.createObjectBuilder()
//         .add("type", "tick")
//         .add("frequency", simulation_frequency)
//         .add("start_time", start_time)
//         .add("frame_times", frame_times).build();
        
//         JsonObjectBuilder vehicles_builder = Json.createObjectBuilder();

//         for (HashMap.Entry<Long, CarInfo> entry : cars.entrySet()){
//             long id = entry.getKey();
//             CarInfo info = entry.getValue();
//             JsonArrayBuilder b_pl_x = Json.createArrayBuilder();
//             JsonArrayBuilder b_pl_y = Json.createArrayBuilder();
//             JsonArrayBuilder b_x = Json.createArrayBuilder();
//             JsonArrayBuilder b_y = Json.createArrayBuilder();
//             JsonArrayBuilder b_orientation = Json.createArrayBuilder();
//             JsonArrayBuilder b_gas = Json.createArrayBuilder();
//             JsonArrayBuilder b_brakes = Json.createArrayBuilder();
//             JsonArrayBuilder b_steering = Json.createArrayBuilder();

//             for (Point2D p : info.planned_trajectory){
//                 b_pl_x.add(p.getX());
//                 b_pl_y.add(p.getY());
//             }

//             for (SimulationFrame frame : frames){
//                 if (frame.time >= info.start_time){
//                     CarFrame car_frame = frame.car_frames.get(id);
//                     if (car_frame == null){
//                         b_x.add(0);
//                         b_y.add(0);
//                         b_orientation.add(0);
//                         b_gas.add(0);
//                         b_brakes.add(0);
//                         b_steering.add(0);
//                     } else {
//                         b_x.add(car_frame.x);
//                         b_y.add(car_frame.y);
//                         b_orientation.add(car_frame.orientation);
//                         b_gas.add(car_frame.gas);
//                         b_brakes.add(car_frame.brakes);
//                         b_steering.add(car_frame.steering);
//                     }
//                 }
//             }
//             JsonObjectBuilder car_obj = Json.createObjectBuilder();
//             car_obj.add(CarEntries.NAME.key_name, info.name);
//             car_obj.add(CarEntries.START_TIME.key_name, info.start_time);
//             car_obj.add(CarEntries.CONFIG.key_name, info.config);
//             car_obj.add(CarEntries.TRAJ_X.key_name, b_pl_x.build());
//             car_obj.add(CarEntries.TRAJ_Y.key_name, b_pl_y.build());
//             car_obj.add(CarEntries.POS_X.key_name, b_x.build());
//             car_obj.add(CarEntries.POS_Y.key_name, b_y.build());
//             //car_obj.add(CarEntries.ORIENTATION.key_name, b_orientation.build());
//             //car_obj.add(CarEntries.GAS.key_name, b_gas.build());
//             //car_obj.add(CarEntries.BRAKES.key_name, b_brakes.build());
//             //car_obj.add(CarEntries.STEERING.key_name, b_steering.build());

//             vehicles_builder.add(Long.toString(id), car_obj);
//         }

//         Json.createArrayBuilder();

//         JsonObject result = Json.createObjectBuilder()
//         .add(Entries.TYPE.key_name, "result")
//         .add(Entries.FRAME_INFO.key_name, time_info)
//         .add(Entries.MAP.key_name, map)
//         //.add(Entries.CONFIG.key_name, config)
//         .add(Entries.VEHICLES.key_name, vehicles_builder.build()).build();

//         try {
//             File file = fileSystem.getPath("results", result_name + ".json");
//             file.getParentFile().mkdirs();
//             file.createNewFile();
//             JsonWriter writer = Json.createWriter(new BufferedOutputStream(new FileOutputStream(file)));
//             writer.writeObject(result);
//             writer.close();
//         } catch(Exception e){
//             System.out.println("Could not export Simulation result for simulation: " + name+":");
//             e.printStackTrace();
//         }
//     }


//     public void import_result(String file_name) throws Exception {
//         JsonObject data = FileSystem.getJson(fileSystem.getPath("results", file_name + ".json"));
//         for (Entries entry : Entries.values()){
//             if (entry.required){
//                 if(!data.containsKey(entry.key_name)){
//                     throw new Exception("Result file missing entry: " + entry.key_name + ": " + entry.format);
//                 }
//             }
//         }
//         if (!data.getString(Entries.TYPE.key_name, "").equalsIgnoreCase("result")){
//             throw new Exception("File loaded as result not of result type.");
//         }


//         JsonObject frame_info = data.getJsonObject(Entries.FRAME_INFO.key_name);
//         JsonArray frame_times = frame_info.getJsonArray("frame_times");
//         if (frame_times == null || frame_times.size() == 0)
//             throw new Exception("Result file with missing/invalid frame_times.");
        
//         frames = new ArrayList<>(frame_times.size());
//         for (JsonValue v : frame_times){
//             if (v.getValueType() != ValueType.NUMBER)
//                 throw new Exception("Expected timestamp (number) as result frame_times.");
            
//             frames.add(new SimulationFrame(((JsonNumber)v).longValueExact()));
//         }

//         JsonNumber fr = frame_info.getJsonNumber("frequency");
//         simulation_frequency = fr != null ? fr.intValue() : 0;
//         if (simulation_frequency <= 0)
//             throw new Exception("Invalid result simulation frequency.");
        
//         if (!frame_info.getString("type", "").equals("tick"))
//             throw new Exception("Invalid result frame time type.");

//         map = data.getString(Entries.MAP.key_name, "");
//         if (map.length() == 0)
//             throw new Exception("Invalid result map name.");

//         //CONFIG

//         JsonObject vehicles = data.getJsonObject(Entries.VEHICLES.key_name);
        
//         cars = new HashMap<Long, CarInfo>();
//         for (Map.Entry<String,JsonValue> e : vehicles.entrySet()){
//             long id = Long.parseLong(e.getKey());
//             if (e.getValue().getValueType() != ValueType.OBJECT)
//                 throw new Exception("Invalid result vehicle entry (not object).");
//             cars.put(id, new CarInfo((JsonObject) e.getValue(), frames, id));
//         }

//         compute_span();
//     }
// }