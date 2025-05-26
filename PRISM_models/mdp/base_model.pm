mdp

const max_power_budget = 30;
const yolov8n_power = 2;
const yolov8x_power = 14;
const yolov8n_accuracy = 37;
const yolov8x_accuracy = 54;
const max_num_obs = 10;

formula p_last_object = 1/(max_num_obs-obs_idd);

module main
    power_left: [0..max_power_budget] init power_left_init;
    obs_idd: [0..max_num_obs] init obs_idd_init;
    are_all_objects_visited: [0..1] init are_all_objects_visited_init; // treated as boolean

    [config0] are_all_objects_visited = 0 & power_left >= yolov8n_power -> 
        (1-p_last_object) : (are_all_objects_visited' = 0) & (power_left'=power_left-yolov8n_power) & (obs_idd'=obs_idd+1) +
        (p_last_object) : (are_all_objects_visited' = 1) & (power_left'=power_left-yolov8n_power) & (obs_idd'=obs_idd+1);
    [config0] are_all_objects_visited = 0 & power_left < yolov8n_power -> (obs_idd'=obs_idd); // to avoid deadlocks, add this option without the power for it
    [config1] are_all_objects_visited = 0 & power_left >= yolov8x_power -> 
        (1-p_last_object) : (are_all_objects_visited' = 0) & (power_left'=power_left-yolov8x_power) & (obs_idd'=obs_idd+1) +
        (p_last_object) : (are_all_objects_visited' = 1) & (power_left'=power_left-yolov8x_power) & (obs_idd'=obs_idd+1);
endmodule

rewards "accuracy"
    [config0] power_left >= yolov8n_power : yolov8n_accuracy;
    [config1] power_left >= yolov8x_power : yolov8x_accuracy;
endrewards
