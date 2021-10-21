import numpy as np
import collections as col


def make_observaton(raw_obs):
    names = ['angle', 'focus','speedX', 'speedY', 'speedZ', \
        'opponents','rpm','track','wheelSpinVel', 'img']
    Observation = col.namedtuple('Observaion', names)
    default_speed = 1
    return Observation(
        angle=np.array(raw_obs['angle'], dtype=np.float32),
        focus=np.array(raw_obs['focus'], dtype=np.float32)/200.,
        speedX=np.array(raw_obs['speedX'], dtype=np.float32) / default_speed,
        speedY=np.array(raw_obs['speedY'], dtype=np.float32) / default_speed,
        speedZ=np.array(raw_obs['speedZ'], dtype=np.float32) / default_speed,
        opponents=np.array(raw_obs['opponents'], dtype=np.float32) / 200.,
        rpm=np.array(raw_obs['rpm'], dtype=np.float32),
        track=np.array(raw_obs['track'], dtype=np.float32) / 200.,
        wheelSpinVel=np.array(raw_obs['wheelSpinVel'], dtype=np.float32),
        img=np.array(raw_obs['img'], dtype=np.uint8) # change
        )


def zero_observation():
    raw_observation = {}
    raw_observation["angle"] = 0
    raw_observation["focus"] = [0] * 5
    raw_observation["speedX"] = 0
    raw_observation["speedY"] = 0
    raw_observation["speedZ"] = 0
    raw_observation["opponents"] = [0] * 36
    raw_observation["rpm"] = 0
    raw_observation["track"] = [0] * 19
    raw_observation["wheelSpinVel"] = 0
    raw_observation["img"] = 0

    return raw_observation

def normalize(affordance):
    affordance_range = [(-2.5, 2.5), (0, 1), (-7, 0), (-2, 3.5), (0, 7), \
        (0, 75), (0, 75), (-9.5, 0), (-5.5, 0), (0, 5.5), (0, 9.5), \
        (0, 75), (0, 75), (0, 75)]
    normalized_affordance = affordance
    for i, zipped in enumerate(zip(affordance, affordance_range)):
        value, value_range = zipped
        normalized_value =  (value - value_range[0]) / (value_range[1] - value_range[0])
        normalized_affordance[i] = min(1, max(0, normalized_value))

    return normalized_affordance


def observation_to_affordance(obs):
    angle = obs.angle
    speedX = obs.speedX
    img = obs.img
    track = obs.track

    # double check that the sensor configuration string in snakeoil 3 is set to 
    #a= "-90 -75 -60 -45 -30 -20 -15 -10 -5 0 5 10 15 20 30 45 60 75 90"

    # Default Values:
    # on-mark affordance:
    toMarking_L = -7
    toMarking_M = 3.5
    toMarking_R = 7
    dist_L = 75
    dist_R = 75
    # in-lane affordance:
    toMarking_LL = -9.5
    toMarking_ML = -5.5
    toMarking_MR = 5.5
    toMarking_RR = 9.5
    dist_LL = 75
    dist_MM = 75
    dist_RR = 75

    left_edge = np.cos(angle) * track[0]
    right_edge = np.cos(angle) * track[-1]
    track_width = left_edge + right_edge
    # normalize track width to 12 (3x4m)
    scaling_factor = 12 / track_width
    left_edge *= scaling_factor
    right_edge *= scaling_factor
    inLane = False
    
    if right_edge < 2.75:
        inLane = True
        lane = "right"
        toMarking_RR = 9.5
        toMarking_LL = right_edge - 8
        toMarking_ML = right_edge - 4
        toMarking_MR = right_edge
    elif left_edge < 2.75:
        inLane = True
        lane = "left"
        toMarking_LL = -9.5
        toMarking_RR = -left_edge + 8
        toMarking_ML = -left_edge
        toMarking_MR = -left_edge + 4
    elif left_edge > 4 and right_edge > 4:
        inLane = True
        lane = "center"
        toMarking_LL = -left_edge
        toMarking_RR = right_edge
        toMarking_ML = -left_edge + 4
        toMarking_MR = right_edge - 4
    
    if not inLane:
        if right_edge > 2.75:
            lane = "center-right"
            toMarking_L = right_edge - 8
            toMarking_M = right_edge - 4
            toMarking_R = right_edge
        elif left_edge > 2.75:
            lane = "center-left"
            toMarking_L = -left_edge
            toMarking_M = -left_edge + 4
            toMarking_R = -left_edge + 8

    affordance = [angle, speedX, \
        toMarking_L, toMarking_M, toMarking_R, dist_L, dist_R, \
        toMarking_LL, toMarking_ML, toMarking_MR, toMarking_RR, \
        dist_LL, dist_MM, dist_RR]
    
    return normalize(affordance)



raw_obs = zero_observation()
obs = make_observaton(raw_obs)
affordance = observation_to_affordance(obs)
print(affordance)

# components: 
# 1. system detection
# 2. on marking system
# 3. in lane system
# 4. normalization