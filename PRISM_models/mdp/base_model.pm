mdp

const max_power_budget = 20;
const yolov8n_power = 1;
const yolov8x_power = 2;
const yolov8n_accuracy = 3;
const yolov8x_accuracy = 4;
const max_num_obs = 10;

formula p_last_object = 1/(max_num_obs-obs_idd);

module main
    power_left: [0..max_power_budget] init max_power_budget;
    obs_idd: [0..max_num_obs] init 0;
    are_all_objects_visited: [0..1] init 0; // treated as boolean

    [config0] are_all_objects_visited = 0 & power_left >= yolov8n_power -> 
        (1-p_last_object) : (are_all_objects_visited' = 0) & (power_left'=power_left-yolov8n_power) & (obs_idd'=obs_idd+1) +
        (p_last_object) : (are_all_objects_visited' = 1) & (power_left'=power_left-yolov8n_power) & (obs_idd'=obs_idd+1);
    [config1] are_all_objects_visited = 0 & power_left >= yolov8x_power -> 
        (1-p_last_object) : (are_all_objects_visited' = 0) & (power_left'=power_left-yolov8x_power) & (obs_idd'=obs_idd+1) +
        (p_last_object) : (are_all_objects_visited' = 1) & (power_left'=power_left-yolov8x_power) & (obs_idd'=obs_idd+1);
endmodule

rewards "accuracy"
    [config0] true : yolov8n_accuracy;
    [config1] true : yolov8x_accuracy;
endrewards
