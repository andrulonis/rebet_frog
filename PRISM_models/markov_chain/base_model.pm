dtmc

const double max_power_budget = 30.0;
const double yolov8n_power = 1.5;
const double yolov8x_power = 14.25;
const double yolov8n_accuracy = 37.0;
const double yolov8x_accuracy = 53.75;
const max_num_obs = 10;

formula p_last_object = 1/(max_num_obs-obs_idd);

module camera
    [] ((detect_model_name = 0 & power_left >= yolov8n_power) | (detect_model_name = 1 & power_left >= yolov8x_power)) & are_all_objects_visited = 0 ->
        (1-p_last_object) : (obs_idd'=obs_idd+1) +
        p_last_object : (obs_idd'=obs_idd+1) & (are_all_objects_visited'=1);
endmodule

rewards "accuracy"
    [] detect_model_name = 0 : yolov8n_accuracy;
    [] detect_model_name = 1 : yolov8x_accuracy;
endrewards

rewards "power"
    [] detect_model_name = 0 : yolov8n_power;
    [] detect_model_name = 1 : yolov8x_power;
endrewards
