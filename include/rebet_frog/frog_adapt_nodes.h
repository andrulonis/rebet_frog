  #pragma once

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rebet/adapt_node.h"
#include "rebet/rebet_utilities.hpp"

#include "rebet_frog/frog_constants.hpp"

#include "rebet_msgs/msg/qr.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace BT
{

class AdaptPictureRateExternal: public AdaptOnConditionOnStart<std::string>
{
  public:
    using QR_MSG = rebet_msgs::msg::QR;
    using KV_MSG = diagnostic_msgs::msg::KeyValue;

    AdaptPictureRateExternal(const std::string& name, const NodeConfig& config) : AdaptOnConditionOnStart<std::string>(name, config, AdaptationTarget::RosParameter, AdaptationType::External)
    {
      _condition_default = true;
      
      // TODO: Make this not ugly
      std::string param_values_string;
      std::string param_name_string;
      std::string node_name;
      getInput(ADAP_OPT, param_values_string);
      getInput(ADAP_SUB, param_name_string);
      getInput(ADAP_LOC, node_name);

      // Expected structure of adaptation_options string is "[minrate]-[maxrate],[topicname];[minrate-maxrate],[topicname]"

      // auto all_param_names = splitString(param_name_string, ';');

      // auto all_param_pairs = splitString(param_values_string, ';');
      // for (auto param_pair : all_param_pairs) {
      //   auto split_pair = splitString(param_pair, ',');
      //   auto rate_limits = split_pair[0];
      //   auto topic = split_pair[1];
      //   auto split_rates = splitString(rate_limits, '-');
      //   auto min_rate = convertFromString<int>(split_rates[0]);
      //   auto max_rate = convertFromString<int>(split_rates[1]);
      //   std::vector<int> rates;
        
      //   for (int i = min_rate; i <= max_rate; ++i) {
      //     rates.push_back(i);
      //   }

      //   aal_msgs::msg::AdaptationOptions rate_param = aal_msgs::msg::AdaptationOptions();
      //   rate_param.name = all_param_names[0];
      //   rate_param.node_name = node_name;
      //   rate_param.adaptation_target_type = static_cast<int8_t>(adaptation_target_);

      //   for (int val : rates) {
      //     rclcpp::ParameterValue par_val = rclcpp::ParameterValue(val);
      //     rate_param.possible_values.push_back(par_val.to_value_msg());
      //   }

      //   aal_msgs::msg::AdaptationOptions topic_param = aal_msgs::msg::AdaptationOptions();
      //   topic_param.name = all_param_names[1];
      //   topic_param.node_name = node_name;
      //   topic_param.adaptation_target_type = static_cast<int8_t>(adaptation_target_);

      //   std::string string_val = convertFromString<std::string>(topic);
      //   rclcpp::ParameterValue par_val = rclcpp::ParameterValue(string_val);
      //   topic_param.possible_values.push_back(par_val.to_value_msg());

      //   _var_params.push_back(rate_param); //vector of VariableParameter   
      //   _var_params.push_back(topic_param); //vector of VariableParameter 
      // }

      auto all_param_values = splitString(param_values_string, ';');
      auto rate_param_values_string = all_param_values[0];
      auto rate_param_values = splitString(rate_param_values_string, ',');
      auto topic_param_values_string = all_param_values[1];
      auto topic_param_values = splitString(topic_param_values_string, ',');

      auto param_names = splitString(param_name_string, ';');

      aal_msgs::msg::AdaptationOptions rate_variable_param = aal_msgs::msg::AdaptationOptions();
      aal_msgs::msg::AdaptationOptions topic_variable_param = aal_msgs::msg::AdaptationOptions();

      
      rate_variable_param.name = param_names[0];
      rate_variable_param.node_name = node_name;
      rate_variable_param.adaptation_target_type = static_cast<int8_t>(adaptation_target_);
      for (const StringView& val : rate_param_values) {
        int int_val = convertFromString<int>(val);
        rclcpp::ParameterValue par_val = rclcpp::ParameterValue(int_val);
        rate_variable_param.possible_values.push_back(par_val.to_value_msg());
      }

      topic_variable_param.name = param_names[1];
      topic_variable_param.node_name = node_name;
      topic_variable_param.adaptation_target_type = static_cast<int8_t>(adaptation_target_);
      for (const StringView& val : topic_param_values) {
        std::string string_val = convertFromString<std::string>(val);
        bool bool_val = string_val == OG_CAMERA_TOPIC;
        rclcpp::ParameterValue par_val = rclcpp::ParameterValue(bool_val);
        topic_variable_param.possible_values.push_back(par_val.to_value_msg());
      }

      _var_params.push_back(rate_variable_param); //vector of VariableParameter   
      _var_params.push_back(topic_variable_param); //vector of VariableParameter   

      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.  
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnConditionOnStart::providedPorts();

      PortsList child_ports =  {
        InputPort<std::vector<std::string>>(ADAP_SUB, "The name of the thing you adapt"), //overwrite to have multiple.
        InputPort<double>(POW_IN,"the power metric"),
        OutputPort<int>(OUT_PIC,"chosen pic_rate"),
        OutputPort<std::string>(OUT_CAM,"current camera topic"),
        InputPort<std::vector<double>>(PICTASK_IN,"the det obj task metric"),
        InputPort<int>(TOT_OBS, "how many obstacles there are"),
        InputPort<rebet::SystemAttributeValue>(IN_LIGHT,"lighting message wrapped in a systemattributevalue instance"),     
        InputPort<std::vector<QR_MSG>>(TASK_QRS_IN, "the task qrs"),   
      };
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual std::vector<QR_MSG> collect_qrs()
    {
      std::vector<QR_MSG> current_qrs;
      auto qr_res = getInput(TASK_QRS_IN, current_qrs);
      return current_qrs;
    }

    virtual std::vector<KV_MSG> collect_context()
    {
      std::vector<KV_MSG> current_context = {};
      int num_obstacles;
      rebet::SystemAttributeValue current_lighting;
      std::vector<double> pic_task;
      double remaining_power_budget;

      auto obstacles_res = getInput(TOT_OBS, num_obstacles);
      auto lighting_res = getInput(IN_LIGHT, current_lighting);
      auto pick_task_res = getInput(PICTASK_IN, pic_task);
      auto pow_budget_res = getInput(POW_IN, remaining_power_budget);
      
      if (obstacles_res) {
        auto kv_obstacles = KV_MSG();
        kv_obstacles.key = "obstacles";
        kv_obstacles.value = std::to_string(num_obstacles);
        current_context.push_back(kv_obstacles);
      }

      if (lighting_res) {
        auto kv_lighting = KV_MSG();
        kv_lighting.key = "darkness";
        kv_lighting.value = std::to_string(current_lighting.get<rebet::SystemAttributeType::ATTRIBUTE_FLOAT>().data);
        current_context.push_back(kv_lighting);
      }

      if (pick_task_res) {
        auto kv_obs_detected = KV_MSG();
        kv_obs_detected.key = "obs_detected";
        kv_obs_detected.value = std::to_string(pic_task[0]);
        current_context.push_back(kv_obs_detected);  

        auto kv_pics_taken = KV_MSG();
        kv_pics_taken.key = "pics_taken";
        kv_pics_taken.value = std::to_string(pic_task[1]);
        current_context.push_back(kv_pics_taken);
      }

      if (pow_budget_res) {
        auto kv_power_budget = KV_MSG();
        kv_power_budget.key = "power_budget";
        kv_power_budget.value = std::to_string(remaining_power_budget);
        current_context.push_back(kv_power_budget);
      }

      return current_context;
    }

    virtual bool evaluate_condition()
    {
      setOutput(OUT_PIC, _current_pic_rate);
      setOutput(OUT_CAM, current_image_feed);
      return true;
    }

    virtual double utility_of_adaptation(rcl_interfaces::msg::Parameter ros_parameter) override
    {
      std::cout << "util_of_adap_cam_rate" << std::endl;
      auto parameter_object = rclcpp::ParameterValue(ros_parameter.value);


      setOutput(OUT_PIC, 1);
      setOutput(OUT_CAM, OG_CAMERA_TOPIC);
      return 0.0;
    }

    private:
      static constexpr const char* POW_IN = "in_power";
      static constexpr const char* PICTASK_IN = "in_pictask";
      static constexpr const char* TOT_OBS = "obstacles_total";
      static constexpr const char* OUT_PIC = "out_pic_rate";
      static constexpr const char* OUT_CAM = "out_cam_top";
      static constexpr const char* ROT = "rotations_done";
      static constexpr const char* IN_LIGHT = "lighting_in";
      static constexpr const char* TASK_QRS_IN = "in_task_qrs";

      std::string ALT_CAMERA_TOPIC = "/corner_camera/image_raw";
      std::string OG_CAMERA_TOPIC = "/camera/image_noisy";

      rebet::SystemAttributeValue _light_attribute;
      std::string detected = std::string(OBJECT_DETECTED_STRING);
      std::string current_image_feed;
      int _current_pic_rate = START_PIC_RATE;
      const int IND_PARAM_TOPIC = 1;
      const int IND_PARAM_NUM = 0;

      int _max_pic_rate = START_PIC_RATE + PIC_INCREMENT;
      int rotations_done = 0;
      int _pictures_taken;
      double _power_consumed;

      int _obs_taken_pictures_of = 0;
      FixedQueue<std::string, PIC_INCREMENT> objs_detected;
};

class FromExploreToIdentify: public AdaptOnConditionOnStart<int>
{
  public:

    FromExploreToIdentify(const std::string& name, const NodeConfig& config) : AdaptOnConditionOnStart<int>(name, config, AdaptationTarget::LifecycleTransition, AdaptationType::Internal)
    {
      _condition_default = true;
      registerAdaptations();
    }


    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnConditionOnStart::providedPorts();

      PortsList child_ports =  { 
        InputPort<std::vector<std::string>>(ADAP_LOC, "Where the thing you adapt is located") //Overwrite the location to have multiple node names.
      };
      child_ports.merge(base_ports);

      return child_ports;
    }

    
    virtual bool evaluate_condition() override
    {
      _internal_adaptations = {};
      
      std::vector<std::string> node_names;
      getInput(ADAP_LOC, node_names);
      lifecycle_msgs::msg::Transition activate_transition;
      activate_transition.id = 3; //Activate transition

      lifecycle_msgs::msg::Transition shutdown_transition;
      shutdown_transition.id = 7; //Shutdown transition

      lifecycle_msgs::msg::Transition configure_transition;
      configure_transition.id = 1; //Configure transition

      aal_msgs::msg::Adaptation adap, adaptwo, adaptthree;

      adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::LifecycleTransition);
      adaptwo.adaptation_target = static_cast<int8_t>(AdaptationTarget::LifecycleTransition);
      adaptthree.adaptation_target = static_cast<int8_t>(AdaptationTarget::LifecycleTransition);

      adap.lifecycle_adaptation = activate_transition;
      adap.node_name = node_names[0];

      adaptwo.lifecycle_adaptation = shutdown_transition;
      adaptwo.node_name = node_names[1];

      adaptthree.lifecycle_adaptation = configure_transition;
      adaptthree.node_name = node_names[0];

      _internal_adaptations.push_back(adaptwo);
      _internal_adaptations.push_back(adaptthree);
      _internal_adaptations.push_back(adap);

      return true;

    }
  private:

};


class AdaptPictureRateInternal: public AdaptOnConditionOnStart<int>
{
  public:

    AdaptPictureRateInternal(const std::string& name, const NodeConfig& config) : AdaptOnConditionOnStart<int>(name, config, AdaptationTarget::RosParameter, AdaptationType::Internal)
    {
      _condition_default = true;
      registerAdaptations();
      _pictures_taken = 0;
      _power_consumed = 0.0;
      current_image_feed = OG_CAMERA_TOPIC;



      
      

      // auto resetCallback = [this](TimePoint timestamp, const TreeNode& node,
      //                                   NodeStatus prev, NodeStatus status) {
      //     std::cout << "\n\n STATUS CHANGED CALLBACK CALLED STATUS IS " << status << std::endl;
      //     if ((status == NodeStatus::IDLE))
      //     {

      //       this->_pictures_taken = 0;
      //       this->_power_consumed = 0.0;
      //       this->_obs_taken_pictures_of = 0;
      //     }
      //   };
      // this->subscribeToStatusChange(std::move(resetCallback));

      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.  
    }

    void resetTracking()
    {
      rotations_done+=1;

      setOutput(ROT, rotations_done);
        std::cout << "\n\n\n RRESET TRACKING \n\n\n" << std::endl;
      _pictures_taken = 0;
      _power_consumed = 0.0;
      _obs_taken_pictures_of = 0;
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnConditionOnStart::providedPorts();

      PortsList child_ports =  {
        InputPort<std::vector<std::string>>(ADAP_SUB, "The name of the thing you adapt"), //overwrite to have multiple.
        InputPort<double>(POW_IN,"the power metric"),
        OutputPort<int>(OUT_PIC,"chosen pic_rate"),
        OutputPort<std::string>(OUT_CAM,"current camera topic"),
        InputPort<std::vector<double>>(PICTASK_IN,"the det obj task metric"),
        InputPort<int>(TOT_OBS, "how many obstacles there are"),
        InputPort<rebet::SystemAttributeValue>(IN_LIGHT,"lighting message wrapped in a systemattributevalue instance"),
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    void pic_adaptation_msg(int pic_rate)
    {
      std::vector<std::string> param_names;
      std::string node_name;


      _current_pic_rate = pic_rate;
      getInput(ADAP_SUB, param_names);
      getInput(ADAP_LOC, node_name);

      std::string param_name = param_names[IND_PARAM_NUM];
      


      aal_msgs::msg::Adaptation adap;
      rclcpp::Parameter adap_param = rclcpp::Parameter(param_name,rclcpp::ParameterValue(pic_rate));

      std::cout << "type name " << adap_param.get_type_name() << std::endl;

      adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::RosParameter);
      adap.parameter_adaptation = adap_param.to_parameter_msg();
      adap.node_name = node_name;

      _internal_adaptations.push_back(adap);

    }

    bool change_camera_feed(std::string new_topic_name)
    {
      if(current_image_feed == new_topic_name){ return false; }
      std::vector<std::string> param_names;
      std::string node_name;

      getInput(ADAP_SUB, param_names);
      getInput(ADAP_LOC, node_name);

      std::string param_name = param_names[IND_PARAM_TOPIC];
      
      aal_msgs::msg::Adaptation adap;
      
      rclcpp::Parameter adap_param = rclcpp::Parameter(param_name,rclcpp::ParameterValue(new_topic_name));

      std::cout << "type name " << adap_param.get_type_name() << std::endl;

      adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::RosParameter);
      adap.parameter_adaptation = adap_param.to_parameter_msg();
      adap.node_name = node_name;

      _internal_adaptations.push_back(adap);

      current_image_feed = new_topic_name;

      return true;

    }


    bool increased_pic_rate()
    {
      _internal_adaptations = {};

      if(_current_pic_rate != _max_pic_rate)
      {
        int new_pic_rate = _current_pic_rate + PIC_INCREMENT;
        return set_pic_rate(new_pic_rate);
      } 

      return false;
      
    }

    bool set_pic_rate(int new_pic_rate)
    {
      _internal_adaptations = {};
      if(new_pic_rate != _current_pic_rate)
      {
        pic_adaptation_msg(new_pic_rate);
        return true;
      }

      return false;

    }

    virtual bool evaluate_condition() override
    {
      setOutput(OUT_PIC, _current_pic_rate);
      setOutput(OUT_CAM, current_image_feed);

      double remaining_power_budget, num_objects_detected, objects_visited;
     
      auto pow_budget_res = getInput(POW_IN, remaining_power_budget);
      auto curr_light_res = getInput(IN_LIGHT,_light_attribute);
      std::vector<double> task_metrics;
      
      auto task_res = getInput(PICTASK_IN, task_metrics);



      int obs_num;
      getInput(TOT_OBS, obs_num);

      if(curr_light_res)
      {
        float current_darkness = _light_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_FLOAT>().data;
        std::cout << "\n\n\nin eval condition darkness\n\n\n" << current_darkness << std::endl;
      }
      else
      {
        std::cout << "\n\n\nin eval condition no lightingg\n\n\n" << std::endl;
      }

      if(pow_budget_res)
      {
        std::cout << "\n\n\nin eval condition remaining pow\n\n\n" << remaining_power_budget << std::endl;
      }

      if(task_res)
      {
        num_objects_detected = task_metrics[0];
        objects_visited = task_metrics[1]; //this is now pictures taken..

        std::cout << "\n\n\nobjects detected \n\n\n" << num_objects_detected << std::endl;
        std::cout << "\n\n\nobjects visited \n\n\n" << objects_visited << std::endl;

      }

      if(task_res && pow_budget_res && curr_light_res)
      {
        float current_darkness = _light_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_FLOAT>().data;
        if(remaining_power_budget < 0.0)
        {
          return change_camera_feed(ALT_CAMERA_TOPIC);
        }
        if((current_darkness) > 0.70)
        {
          std::cout << "got this far darkness" << std::endl;

            double total_pics_quota = (double)obs_num * 5; //5 is the repeat in the BT specified.

            double pics_left = total_pics_quota - objects_visited; //quota minus conusmed.
            double extra_power_consumed = PIC_INCREMENT * DETECTION_AVG_POW;

            double power_needed_after = (double)pics_left * DETECTION_AVG_POW; //Power still necessary to take 5 picture of each obj

            double power_to_be_used = power_needed_after + extra_power_consumed;

            std::cout << "power to be used " << power_to_be_used << std::endl;
            if(remaining_power_budget >= power_to_be_used && remaining_power_budget > 0.0) 
            {
              //While there's still budget we prefer getting the more useful detection from the robot
              return increased_pic_rate() || change_camera_feed(OG_CAMERA_TOPIC); //The latter is just in case the ext. camera is in use right now
            }
            else
            {
              //if there's no budget and its noisy, we use the alternative camera.
              return change_camera_feed(ALT_CAMERA_TOPIC);
            }
          
        }
        else { 
          //If it isn't noisy, we conservatively use the robot's camera.
          return (set_pic_rate(START_PIC_RATE) || change_camera_feed(OG_CAMERA_TOPIC));
          }
      }

      return false;
    }
  private:
      static constexpr const char* POW_IN = "in_power";
      static constexpr const char* PICTASK_IN = "in_pictask";

      static constexpr const char* TOT_OBS = "obstacles_total";
      static constexpr const char* OUT_PIC = "out_pic_rate";
      static constexpr const char* OUT_CAM = "out_cam_top";
      static constexpr const char* ROT = "rotations_done";
      static constexpr const char* IN_LIGHT = "lighting_in";

      std::string ALT_CAMERA_TOPIC = "/corner_camera/image_raw";
      std::string OG_CAMERA_TOPIC = "/camera/image_noisy";

      rebet::SystemAttributeValue _light_attribute;
      std::string detected = std::string(OBJECT_DETECTED_STRING);
      std::string current_image_feed;
      int _current_pic_rate = START_PIC_RATE;
      const int IND_PARAM_TOPIC = 1;
      const int IND_PARAM_NUM = 0;

      int _max_pic_rate = START_PIC_RATE + PIC_INCREMENT;
      int rotations_done = 0;
      int _pictures_taken;
      double _power_consumed;

      int _obs_taken_pictures_of = 0;
      FixedQueue<std::string, PIC_INCREMENT> objs_detected;



};

class AdaptMaxSpeedExternal : public AdaptPeriodicallyOnRunning<double>
{
  public:
    using QR_MSG = rebet_msgs::msg::QR;

    AdaptMaxSpeedExternal(const std::string& name, const NodeConfig& config) : AdaptPeriodicallyOnRunning<double>(name, config, AdaptationTarget::RosParameter, AdaptationType::External)
    {
      //Since we are only interested in modifying the x-axis (backwards/forwards) speed, we wrap the adaptation_options given into the required triple of x y theta speeds.
        std::vector<double> param_values;
        std::string param_name;
        std::string node_name;
        getInput(ADAP_OPT, param_values);
        getInput(ADAP_SUB, param_name);
        getInput(ADAP_LOC, node_name);

        
        power_qr_max_value = (double)calculate_power_motion(WAFFLE_MAX_LIN_VEL);
        aal_msgs::msg::AdaptationOptions variable_param = aal_msgs::msg::AdaptationOptions();


        variable_param.name = param_name;
        variable_param.node_name = node_name;
        variable_param.adaptation_target_type = static_cast<int8_t>(adaptation_target_);

        for (double val : param_values) {
            std::vector<double> speed_vector = {val,_default_y_velocity,_default_theta_velocity};
            rclcpp::ParameterValue par_val = rclcpp::ParameterValue(speed_vector); //Here we wrap it with default values
            variable_param.possible_values.push_back(par_val.to_value_msg());
        }
        _var_params.push_back(variable_param); //vector of VariableParameter   

      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.  
    }

    

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptPeriodicallyOnRunning::providedPorts();

      PortsList child_ports =  {
        InputPort<double>(POW_IN,"the power metric"),
        InputPort<double>(SAFE_IN,"the safety metric"),
        InputPort<double>(MOVE_IN,"the movement efficiency"),
        InputPort<std::vector<QR_MSG>>(TASK_QRS_IN, "the task qrs"),
        OutputPort<double>(SPD_OUT,"the current chosen max speed"),
        


              };
      child_ports.merge(base_ports);

      return child_ports;
    }


    bool unsafe_power_hungry(double current_safety, double current_power)
    {
      return ((current_safety < 0.09) || (current_power > 5.0));
    }

    bool safe_lowpower_slow(double current_safety, double current_power, double current_move_eff)
    {
      return (current_safety > 0.15 || current_power < 4.0 || current_move_eff < 0.40);
    }

    bool is_safe(double current_safety)
    {
      return current_safety < PROXIMITY_SAFETY_THRESHOLD;
    }


    virtual std::vector<QR_MSG> collect_qrs()
    {
      std::vector<QR_MSG> current_qrs;
      auto qr_res = getInput(TASK_QRS_IN, current_qrs);
      return current_qrs;
    }

    virtual double utility_of_adaptation(rcl_interfaces::msg::Parameter ros_parameter) override
    {
      std::cout << "util_of_adap_max_speed" << std::endl;
      auto parameter_object = rclcpp::ParameterValue(ros_parameter.value);

      std::vector<double> chosen_speeds = parameter_object.get<std::vector<double>>();



      double chosen_max_speed = chosen_speeds[0];

      setOutput(SPD_OUT, chosen_max_speed);
    
      double current_safety;
      double current_power;
      double current_move;
      auto safe_res = getInput(SAFE_IN, current_safety);
      auto pow_res = getInput(POW_IN, current_power);
      auto move_res = getInput(MOVE_IN, current_move);

 //if(current_power < 4.0 || current_safety > 0.15 || current_move < 0.40)

      if(safe_res && pow_res && move_res)
      {
        //MoveSafely is in violation.
        if(!is_safe(current_safety) && chosen_max_speed > 0.10)
        {
          return 0.0;
        }
        else
        {
          
          double inverse_power = 1/current_power;

          std::cout << "inverse_power " << inverse_power << " currentmove: " << current_move << std::endl; 


          double utility = current_move * inverse_power;

          if(utility < 0.0 || current_move < 0.0 || current_power < 0.0) //can happen if there's no values yet.
          {
            return 0.0;
          }
          return 0.0;
        }

      }
      else
      {
        std::cout << "For some reason a value was not found in the blackboard" << std::endl;  
        return 0.0;
      }

    }
        // std::cout << "I'm here in internal max spd! \n\n" << std::endl;
        // if(safe_res && pow_res && move_res)
        // {
        //   std::cout << "now here curr_safe " << current_safety << " curr_pow " << current_power << std::endl;

        //   if( (current_safety < 0.09) || (current_power > 5.0) )
        //   {
        //     std::cout << "decrease speed here " << std::endl;
            
        //     return decrease_speed();
        //   }
        //   if(current_power < 4.0 || current_safety > 0.15 || current_move < 0.40)
        //   {
        //     std::cout << "increase speed here " << std::endl;

        //     return increase_speed();
        //   }


  private:
    double _default_y_velocity = 0.0; //The robot in question can not move along its y axis independently.
    double _default_theta_velocity = 2.5; //We are not interested in modifying the rotation speed here, but could be extended to do so.
    static constexpr const char* POW_IN = "in_power";
    static constexpr const char* SAFE_IN = "in_safety";
    static constexpr const char* MOVE_IN = "in_movement";
    static constexpr const char* TASK_QRS_IN = "in_task_qrs";
    static constexpr const char* SPD_OUT = "report_speed";
    double PROXIMITY_SAFETY_THRESHOLD = 0.10;
    double power_qr_max_value;


};




class AdaptMaxSpeedInternal : public AdaptPeriodicallyOnRunning<double>
{
  public:

    AdaptMaxSpeedInternal(const std::string& name, const NodeConfig& config) : AdaptPeriodicallyOnRunning<double>(name, config, AdaptationTarget::RosParameter, AdaptationType::Internal)
    {
      //Since we are only interested in modifying the x-axis (backwards/forwards) speed, we wrap the adaptation_options given into the required triple of x y theta speeds.
        std::vector<double> param_values;
        std::string param_name;
        std::string node_name;
        getInput(ADAP_OPT, param_values);
        getInput(ADAP_SUB, param_name);
        getInput(ADAP_LOC, node_name);

        

        // aal_msgs::msg::AdaptationOptions variable_param = aal_msgs::msg::AdaptationOptions();


        // variable_param.name = param_name;
        // variable_param.node_name = node_name;
        // variable_param.adaptation_target_type = static_cast<int8_t>(adaptation_target_);

        // for (double val : param_values) {
        //     std::vector<double> speed_vector = {val,_default_y_velocity,_default_theta_velocity};
        //     rclcpp::ParameterValue par_val = rclcpp::ParameterValue(speed_vector); //Here we wrap it with default values
        //     variable_param.possible_values.push_back(par_val.to_value_msg());
        // }
        // _var_params.push_back(variable_param); //vector of VariableParameter   

      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.  
    }

    void speed_adaptation_msg(double max_speed_value)
    {
      _current_max_speed = max_speed_value;

      std::vector<double> speed_vector = {max_speed_value,_default_y_velocity,_default_theta_velocity};

      std::string param_name;
      std::string node_name;


      getInput(ADAP_SUB, param_name);
      getInput(ADAP_LOC, node_name);


      aal_msgs::msg::Adaptation adap;
      rclcpp::Parameter adap_param = rclcpp::Parameter(param_name,rclcpp::ParameterValue(speed_vector));
      adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::RosParameter);
      adap.parameter_adaptation = adap_param.to_parameter_msg();
      adap.node_name = node_name;

      _internal_adaptations.push_back(adap);

    }

    bool decrease_speed()
    {
      _internal_adaptations = {};

      double new_max_speed = _current_max_speed - SPEED_INCREMENT;

      if((new_max_speed + EPSILON) < MIN_SPEED)
      {
        std::cout << std::setprecision(20) << "New max speed: " << new_max_speed << std::endl;
        std::cout << std::setprecision(20) << "MIN_SPEED: " << MIN_SPEED << std::endl;
        std::cout << "no decrease speed possible to go from _ to _" << _current_max_speed << " " << new_max_speed << std::endl;

        return false;
      }

      speed_adaptation_msg(new_max_speed);

      return true;
    }

    bool increase_speed()
    {
      _internal_adaptations = {};

      double new_max_speed = _current_max_speed + SPEED_INCREMENT;

      if(new_max_speed > MAX_MAX_SPEED)
      {
        return false;
      }

      speed_adaptation_msg(new_max_speed);

      return true;
    }

    virtual bool evaluate_condition() override
    {
      setOutput(SPD_OUT, _current_max_speed);
      if(AdaptPeriodicallyOnRunning::evaluate_condition())
      {
        double current_safety;
        double current_power;
        double current_move;
        auto safe_res = getInput(SAFE_IN, current_safety);
        auto pow_res = getInput(POW_IN, current_power);
        auto move_res = getInput(MOVE_IN, current_move);

        




        std::cout << "I'm here in internal max spd! \n\n" << std::endl;
        if(safe_res && pow_res && move_res)
        {
          std::cout << "now here curr_safe " << current_safety << " curr_pow " << current_power << std::endl;

          if( (current_safety < MIN_LIDAR_RANGE) || (current_power > HIGH_MOVE_POW_CONSUMPTION) )
          {
            std::cout << "decreasing max speed " << std::endl;
            
            return decrease_speed();
          }
          if(current_power < (HIGH_MOVE_POW_CONSUMPTION - 1.0) || current_safety > (MIN_LIDAR_RANGE + 0.05) || current_move < DESIRABLE_SPEED)
          {
            std::cout << "increasing speed here " << std::endl;

            return increase_speed();
          }
        }
      }
      return false;    
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptPeriodicallyOnRunning::providedPorts();

      PortsList child_ports =  {
        InputPort<double>(POW_IN,"the power metric"),
        InputPort<double>(SAFE_IN,"the safety metric"),
        InputPort<double>(MOVE_IN,"the movement efficiency"),
        OutputPort<double>(SPD_OUT,"the current chosen max speed"),
        
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

  private:
    double _default_y_velocity = 0.0; //The robot in question can not move along its y axis independently.
    double _default_theta_velocity = 2.5; //We are not interested in modifying the rotation speed here, but could be extended to do so.
    static constexpr const char* POW_IN = "in_power";
    static constexpr const char* SAFE_IN = "in_safety";
    static constexpr const char* MOVE_IN = "in_movement";
    static constexpr const char* SPD_OUT = "report_speed";

    const double SPEED_INCREMENT = 0.08;
    const double MIN_SPEED = 0.10;
    const double MAX_MAX_SPEED = 0.26;
    const double MIN_LIDAR_RANGE = 0.10;
    const double DESIRABLE_SPEED = 0.40;
    const double HIGH_MOVE_POW_CONSUMPTION = 40.0;
    const double EPSILON = 0.01; //Floating-point tolerance..
    double _current_max_speed = MAX_MAX_SPEED;
};



}   // namespace BT