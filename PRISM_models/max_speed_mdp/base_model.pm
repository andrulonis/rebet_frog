mdp

const c0_speed = 18; // cm/s
const c1_speed = 26; // cm/s

const double unsafe_threshold = 0.1;

formula p_unsafe_short_term = current_safety < unsafe_threshold ? 1 : 1 - (current_safety - unsafe_threshold); // find magic value for the last 1 because usually the safety will be far below 1
formula p_unsafe_long_term = min(1 - (mean_safety - unsafe_threshold), 1 - (unsafe_threshold - mean_safety));

module main
    distance_to_pose : [0..distance_to_pose_init] init distance_to_pose_init; // assumed to be in cm
    is_fast : [0..1] init is_fast_init;
    is_unsafe : [0..1] init is_unsafe_init;
    short_term : [0..1] init short_term_init; // boolean, we want to use a different probability for this adaptation than for future ones.

    [config0] short_term = 1 ->  p_unsafe_short_term : (distance_to_pose'=max(distance_to_pose-c0_speed,0)) & (short_term'=0) & (is_unsafe'=1) & (is_fast'=0) + 
                                (1-p_unsafe_short_term) : (distance_to_pose'=max(distance_to_pose-c0_speed,0)) & (short_term'=0) & (is_unsafe'=0) & (is_fast'=0);
    [config0] short_term = 0 ->  p_unsafe_long_term : (distance_to_pose'=max(distance_to_pose-c0_speed,0)) & (is_unsafe'=1) & (is_fast'=0) + 
                                (1-p_unsafe_long_term) : (distance_to_pose'=max(distance_to_pose-c0_speed,0))& (is_unsafe'=0) & (is_fast'=0);
    [config1] short_term = 1 ->  p_unsafe_short_term : (distance_to_pose'=max(distance_to_pose-c1_speed,0)) & (short_term'=0) & (is_unsafe'=1) & (is_fast'=1) + 
                                (1-p_unsafe_short_term) : (distance_to_pose'=max(distance_to_pose-c1_speed,0)) & (short_term'=0) & (is_unsafe'=0) & (is_fast'=1);
    [config1] short_term = 0 ->  p_unsafe_long_term : (distance_to_pose'=max(distance_to_pose-c1_speed,0)) & (is_unsafe'=1) & (is_fast'=1)  + 
                                (1-p_unsafe_long_term) : (distance_to_pose'=max(distance_to_pose-c1_speed,0))& (is_unsafe'=0) & (is_fast'=1);
endmodule

rewards
    is_unsafe = 1 & is_fast = 1: 1;
    is_unsafe = 0 & is_fast = 0: 1;
endrewards
