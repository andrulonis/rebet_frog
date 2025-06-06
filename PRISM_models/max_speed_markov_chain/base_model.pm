dtmc

const double unsafe_threshold = 0.1;

formula is_safe = current_safety >= unsafe_threshold ? 1 : 0;
formula is_fast = max_velocity_0 > 0.18 ? 1 : 0;
formula p_next_step_safe = mean_safety >= unsafe_threshold ? 1 : 0;

module main    
    safety: [0..1] init is_safe; // 0 - unsafe state, 1 - safe state
    [] true -> p_next_step_safe : (safety'=1) + 1-p_next_step_safe : (safety'=0);
endmodule

rewards "dangerous"
    [] ((is_fast = 1) & (safety = 0)): 1;
endrewards

rewards "fast"
    [] (is_fast = 1): 1;
endrewards
