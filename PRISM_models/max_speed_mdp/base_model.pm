mdp

const adaptation_period = 1; // s
const c0_speed = 18 * adaptation_period; // cm/adaptation loop
const c1_speed = 26 * adaptation_period; // cm/adaptation loop

const double unsafe_threshold = 0.1;

const double penalty = 0.5;


formula p_unsafe_short_term = (current_safety < unsafe_threshold) ? 1 : (current_safety > 1.5*unsafe_threshold ? 0 : (1.5*unsafe_threshold - current_safety) / unsafe_threshold);
formula p_unsafe_long_term = (mean_safety < 0.5*unsafe_threshold) ? 1 : (mean_safety > 1.5*unsafe_threshold ? 0 : (1.5*unsafe_threshold - mean_safety) / unsafe_threshold);

module main
    distance_to_pose : [0..distance_to_pose_init] init distance_to_pose_init; // assumed to be in cm
    is_fast : [0..1] init is_fast_init;
    is_unsafe : [0..1] init is_unsafe_init;
    short_term : [0..1] init short_term_init; // boolean, we want to use a different probability for this adaptation than for future ones

    [config0] short_term = 1 ->  p_unsafe_short_term : (distance_to_pose'=max(distance_to_pose-c0_speed,0)) & (short_term'=0) & (is_unsafe'=1) & (is_fast'=0) + 
                                (1-p_unsafe_short_term) : (distance_to_pose'=max(distance_to_pose-c0_speed,0)) & (short_term'=0) & (is_unsafe'=0) & (is_fast'=0);
    [config0] short_term = 0 ->  p_unsafe_long_term : (distance_to_pose'=max(distance_to_pose-c0_speed,0)) & (is_unsafe'=1) & (is_fast'=0) + 
                                (1-p_unsafe_long_term) : (distance_to_pose'=max(distance_to_pose-c0_speed,0))& (is_unsafe'=0) & (is_fast'=0);
    [config1] short_term = 1 ->  p_unsafe_short_term : (distance_to_pose'=max(distance_to_pose-c1_speed,0)) & (short_term'=0) & (is_unsafe'=1) & (is_fast'=1) + 
                                (1-p_unsafe_short_term) : (distance_to_pose'=max(distance_to_pose-c1_speed,0)) & (short_term'=0) & (is_unsafe'=0) & (is_fast'=1);
    [config1] short_term = 0 ->  p_unsafe_long_term : (distance_to_pose'=max(distance_to_pose-c1_speed,0)) & (is_unsafe'=1) & (is_fast'=1)  + 
                                (1-p_unsafe_long_term) : (distance_to_pose'=max(distance_to_pose-c1_speed,0))& (is_unsafe'=0) & (is_fast'=1);
endmodule

// The reward represents the number of seconds passed before reaching the goal, which we want to minimise.
// To disincentivise moving fast while unsafe, we add a penalty in seconds. This penalty is not part of the actual system, only part of the model.
rewards "cost"
    is_unsafe = 1 & is_fast = 1: penalty;
    true: adaptation_period;
endrewards
