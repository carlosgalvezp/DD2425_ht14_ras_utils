/**
 * The Node class is meant to serve as a base class for nodes. It will create a private namespaces node handle
 * and will provide a convinient interface for handling ros params.
 */

#ifndef __ROB_BASIC_NODE_H
#define __ROB_BASIC_NODE_H

#include <vector>
#include <ros/ros.h>
#include <boost/variant.hpp>
#include <ras_srv_msgs/Command.h>
#include <ras_utils/ras_names.h>

#define PRINT_PADDING       5
#define NODE_HANDLE_PARAM   "~"

#define ACTIVATE_NODE_TOPIC_NAME  "brain/activate_node"

namespace rob {
    class BasicNode {
        struct Param {
            std::string name;

            enum Type {
                BOOL,
                INT,
                DOUBLE,
                STRING
            };

            boost::variant<bool, int, double, std::string> default_value;
            boost::variant<bool*, int*, double*, std::string*> destination;

            bool no_default;

            template<typename T>
            Param(const std::string & name, T * destination, T * default_value) {
                this->name = name;
                this->destination = destination;
                if(default_value == NULL) {
                    no_default = true;
                } else {
                    no_default = false;
                    this->default_value = *default_value;
                }
            }
        };

    private:
        std::vector<Param> params;
        int longest_name_length;
        const int print_padding;
    protected:
        ros::NodeHandle n;
        // ** Services
        ros::ServiceClient srv_out_;
        ros::ServiceServer srv_in_;
        /**
         * @brief communicates a command to another node
         * @param srv_name the other node's server name
         * @param command the command
         * @return true if successfully sent the message
         */
        bool communicate(const std::string &srv_name, int command)
        {
            srv_out_ = n.serviceClient<ras_srv_msgs::Command>(srv_name);
            ras_srv_msgs::Command message;
            message.request.command = command;
            return srv_out_.call(message);
        }
    public:
        BasicNode(int print_padding = PRINT_PADDING) : n(NODE_HANDLE_PARAM), longest_name_length(0), print_padding(print_padding) {}

        BasicNode(std::string node_handle_param, int print_padding = PRINT_PADDING) : n(node_handle_param), longest_name_length(0), print_padding(print_padding) {}

        /**
         * This functions register a static ros paramater. The node will try to fetch the value from the rosparam server.
         * If the value doesn't exist in the param server, the default value will be sent to the param server and set to the
         * local parameter. The read/write will only occur once (i.e. when this function is called).
         */
        template<typename T>
        void add_param(const std::string & name, T & destination, T default_value) {
            add_param(name, destination, &default_value);
        }

        void add_param(const std::string & name, std::string & destination, char default_value[]) {
            std::string temp(default_value);
            add_param(name, destination, &temp);
        }

        template<typename T>
        void add_param(const std::string & name, T & destination, T * default_value = NULL) {
            params.push_back(Param(name, &destination, default_value));
            init_param(name, destination, default_value);

            if(name.length() > longest_name_length) {
                longest_name_length = name.length();
            }
        }

        void print(const std::string & text) {
             ROS_INFO("%s",text.c_str());
        }

        void print(const std::string & text, const double value, std::string & padding) {
            ROS_INFO((text + ":" + padding + "%f").c_str(), value);
        }

        void print(const std::string & text, const double value) {
            std::string padding;
            padding.assign(print_padding, ' ');
            ROS_INFO((text + ":" + padding + "%f").c_str(), value);
        }
        void print(const std::string & text, const double value1, const double value2) {
            std::string padding;
            padding.assign(print_padding, ' ');
            ROS_INFO((text + ":" + padding + "%f | %f").c_str(), value1, value2);
        }

        void print(const std::vector<std::string> & texts, const std::vector<double> & values) {
            if(texts.size() != values.size()) {
                print("!!! ERROR !!! Vectors in print function have different sizes");
                print("texts, values", texts.size(), values.size());
                return;
            }
            int longest_text_length = 0;
            for(std::string text : texts) {
                if(text.length() > longest_text_length) {
                    longest_text_length = text.length();
                }
            }

            std::string padding;


            for(int i = 0; i < texts.size(); i++) {
                int missing = longest_text_length + print_padding - texts[i].length();
                padding.assign(missing, ' ');
                print(texts[i], values[i], padding);
            }
            ROS_INFO("");
        }


        //TODO: update_param.

        /**
         * Prints all registered params this node is operating on. Will also print the current values of the params.
         */
        void print_params() {

            ROS_INFO("--Params--");

            std::string padding;

            auto bool_to_str = [](bool value) {
                return value ? "true" : "false";
            };

            for(auto param : params) {
                int missing = longest_name_length + print_padding - param.name.length();
                padding.assign(missing, ' ');

                switch(Param::Type(param.destination.which())) {
                case Param::Type::BOOL:
                    print_params<bool>(padding, param, bool_to_str);
                    break;
                case Param::Type::INT:
                    print_params<int>(padding, param);
                    break;
               case Param::Type::DOUBLE:
                    print_params<double >(padding, param);
                    break;
                case Param::Type::STRING:
                    print_params<std::string>(padding, param);
                    break;
                }
            }

            ROS_INFO("");

        }

    private:

        template<class T>
        void print_params(const std::string & padding, const Param &param) {
        std::function<std::string(const T &)> f = [](const T &val){return  boost::lexical_cast<std::string>(val);};
            print_params(padding, param, f);
        }

        template<class T>
        void print_params(const std::string & padding,
                          const Param &param,
                          std::function<std::string(const T &)> type_to_string) {

            std::string info_string = param.name + ":" + padding;

            T * dest = boost::get<T *>(param.destination);
            info_string += type_to_string(*dest);

            if(!param.no_default) {
                T def = boost::get<T>(param.default_value);
                info_string += " (" + type_to_string(def) + ")";
            }
            ROS_INFO("%s",info_string.c_str());
        }

        template<class T>
        void init_param(const std::string & name, T & destination, T * default_value) {
            if(!n.hasParam(name)) {
                if(default_value != NULL) {
                    n.setParam(name, default_value);
                } else {
                    ROS_ERROR("No parameter or default parameter set for: %s", name.c_str());
                    throw std::exception();
                }
            }

            if(!n.getParam(name, destination)) {
                ROS_WARN("Failed to get parameter %s from param server. Falling back to default value.", name.c_str());
                destination = * default_value;
            }
        }
    };
}

#endif
