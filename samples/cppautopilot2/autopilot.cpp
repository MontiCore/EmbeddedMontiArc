#include "autopilot.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>


void Autopilot::init() {
    time = 0;
    true_velocity = 0;
    true_position.x = 0;
    true_position.y = 0;
    true_compass = 0;
    speed_pid = PID(1, 0, 0.2);
}
// Simple Zig-Zag at TARGET_VELOCITY
// void Autopilot::execute(double delta_sec) {

//     // Go to constant 30 km/h
//     double output = speed_pid.compute(delta_sec, true_velocity, TARGET_VELOCITY);
//     output /= 3.6; // Convert to m/s related space
//     set_gas = output / MAX_VEHICLE_ACCEL; // Convert to [0:1] actuator range

//     // Turn in zig-zags
//     set_steering = sin(time)*20;

//     time += delta_sec;
// }

inline double steering_from_radius(double r){
    return 452/r;
}
inline double radius_from_steering(double w){
    return 452/w;
}
inline double max_speed_for_steering(double w){
    return (Autopilot::SAFE_TURN_FACTOR*270)/sqrt(w);
}
inline double max_steering_for_speed(double s){
    auto t = (Autopilot::SAFE_TURN_FACTOR*270)/s;
    return t*t;
}
inline double junction_angle(const Segment &s1, const Segment &s2){
    return acos(dot(s1.dir, s2.dir));
}
inline double alpha_from_junction_angle(double ja){
    return (PId - ja)*0.5;
}
inline double turn_radius_for_turn_distance(double d, double alpha){
    return std::tan(alpha)*d;
}
inline double turn_distance_for_turn_radius(double r, double alpha){
    return r/std::tan(alpha);
}
inline double max_radius_for_junction(double alpha, double max_deviation){
    return max_deviation / ((1.0/std::sin(alpha))-1);
}

inline bool valid_tangent_turn(const Segment &seg, const vec2f64 &dir){
    return dot(seg.normal, dir)*seg.ortho_pos < -0.0001;
}
inline double tangent_turn_radius(const Segment &seg, double ja){
    return seg.dist/std::sin(ja*0.5);
}

void Autopilot::execute(double delta_sec) {
    if (trajectory_x__size != trajectory_y__size) return; // TODO error msg ?

    Segment nearest_segment;
    if (!get_nearest_segment(true_position, nearest_segment)){
        // No trajectory -> Stay in place
        set_steering = 0;
        set_gas = 0;
        set_braking = 1;
        return;
    }
    Segment *target_segment = &nearest_segment;
    Segment *next_segment = nullptr;
    
    
    // Check if behind trajectory start
    Segment proxy_segment;
    if (nearest_segment.id == 0 && nearest_segment.proj_pos < 0) {
        proxy_segment.init_segment(-1, true_position, nearest_segment.start, true_position);
        target_segment = &proxy_segment;
        next_segment = &nearest_segment;
    }
    else if (nearest_segment.dist > ROAD_WIDTH){ // If outside the road => add proxy segment towards the road
        auto proj_point = nearest_segment.start + nearest_segment.dir*nearest_segment.proj_pos;
        proxy_segment.init_segment(nearest_segment.id - 1, true_position, proj_point, true_position);
        target_segment = &proxy_segment;
        next_segment = &nearest_segment;
    }

    // Check if next segment exists if not already set
    Segment segment_after;
    if (next_segment == nullptr){
        auto next_id = target_segment->id + 1;
        if (next_id < trajectory_x__size){
            segment_after.init_segment(next_id, target_segment->end, vec2f64(trajectory_x[next_id], trajectory_y[next_id]), true_position);
            next_segment = &segment_after;
        }
    }

    bool in_turn = false;

    // Check upcoming turn and decide to which segment to align
    if (next_segment != nullptr && target_segment->proj_pos_end < MAX_TURN_SIZE) {
        double max_turn_size = get_max_turn_size(*target_segment, *next_segment);

        // Check if "in turn"
        if (target_segment->proj_pos_end <= max_turn_size) {
            in_turn = true;
            target_segment = next_segment;
            next_segment = nullptr;
        }
    }

    // Eval target speed :
    // Maximum allowed speed if following segment
    double max_speed = MAX_ROAD_SPEED;
    // If Approaching turn: Get required turn radius to not deviate too much from segments
    if (next_segment != nullptr) {
        double max_turn_size = get_max_turn_size(*target_segment, *next_segment);
        double turn_size = std::min(max_turn_size, MAX_TURN_SIZE);
    }
    // From this 
    

    // TODO

    // Align to target segment
}

void Autopilot::follow_segment(const Segment& seg, double target_speed){
    auto car_angle = true_compass * DEG_TO_RADd;
    auto max_steering = std::min(max_steering_for_speed(std::abs(true_velocity)), MAX_STEERING); // In degrees
    auto min_turn_radius = radius_from_steering(max_steering);
    auto car_dir = vec2f64(std::cos(car_angle), std::sin(car_angle));
    auto ja = acos(dot(car_dir, seg.dir));

    // Check if in range for a tangent turn
    if (valid_tangent_turn(seg, car_dir)){
        auto tangent_radius = tangent_turn_radius(seg, ja);
        if (tangent_radius >= min_turn_radius) {
            auto tangent_steering = steering_from_radius(tangent_radius);
            if (tangent_steering > max_steering - )
        }
    }
}

double Autopilot::get_max_turn_size(const Segment& current_segment, const Segment &next_segment){
    // Eval junction turn
    double junction_angle = acos(dot(current_segment.dir, next_segment.dir));
    //double junctionSign = Math.signum(IPM.dot(normal, nextSeg.dir));
    double max_radius = std::numeric_limits<double>::infinity();
    if (junction_angle > 0.000001 || junction_angle < -0.000001){
        // Formula to define the radius of a circle tangent to the 2 segment with distance to junction < ROAD_WIDTH
        max_radius = ROAD_WIDTH / ((1 / cos(junction_angle * 0.5)) - 1);
    }
    return sqrt(ROAD_WIDTH * (ROAD_WIDTH + max_radius * 2));
}

bool Autopilot::get_nearest_segment(const vec2f64& vehicle_pos, Segment &target){
    if (trajectory_x__size <= 0) return false;
    if (trajectory_x__size == 1) {
        target.init_segment(0, vehicle_pos, vec2f64(trajectory_x[0], trajectory_y[0]), vehicle_pos);
        return true;
    }
    vec2f64 point;
    vec2f64 last_point = vec2f64(trajectory_x[0], trajectory_y[0]);;
    Segment seg;
    double curr_best = std::numeric_limits<double>::infinity();
    for (int i = 1; i < trajectory_x__size; i++) {
        auto point = vec2f64(trajectory_x[i], trajectory_y[i]);
        bool is_better = false;

        // Check segment
        seg.init_segment(i, last_point, point, vehicle_pos);
        // check if in segment bounds
        if (seg.proj_pos > 0 && seg.proj_pos < seg.length){
            if (seg.dist < curr_best) {
                curr_best = seg.dist;
                is_better = true;
            }
        }

        // Check point
        double dist = seg.rel_pos.length();
        if (dist < curr_best) {
            curr_best = dist;
            is_better = true;
        }
        if (is_better) target = seg;

        last_point = point;
    }

    // Check if after the last trajectory point
    double dist = (last_point-vehicle_pos).length();
    if (dist < curr_best) target.init_segment(trajectory_x__size-1,vehicle_pos, last_point, vehicle_pos);

    return true;
}


void Segment::init_segment(int id, const vec2f64& start, const vec2f64& end, const vec2f64& vehicle_pos){
    this->id = id;
    this->start = start;
    this->end = end;
    dir = end-start;
    this->length = dir.length();
    this->dir = length < 0.00000001 && length > -0.00000001 ? vec2f64(0,0) : dir / length;
    this->normal = vec2f64(-dir.y, dir.x);
    this->rel_pos = vehicle_pos - start;
    this->proj_pos = dot(rel_pos, dir);
    this->proj_pos_end = length - proj_pos;
    this->ortho_pos = dot(rel_pos, normal);
    this->dist = abs(ortho_pos);
}
void Segment::init_point(const vec2f64& position, const vec2f64& vehicle_pos){
    this->is_point = true;
    this->start = position;
    this->rel_pos = vehicle_pos - position;
    this->dir = normalize(-rel_pos);
    this->proj_pos_end = rel_pos.length();

    double n = nan(nullptr);
    this->end = vec2f64(n);
    this->length = 0;
    this->dist = 0;
}