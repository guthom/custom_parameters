#pragma once

#include <ros/ros.h>
#include <string.h>
#include <map>
#include <vector>
#include <iterator>

#include "parameter.h"

namespace customparameter
{
    class ParameterHandler{

        public:
            //all the different maps
            std::map<std::string, Parameter<XmlRpc::XmlRpcValue> > xmlRpcParameters;

            std::map<std::string, Parameter<int> > intParameters;
            std::map<std::string, Parameter<std::vector<int> > > intVectorParameters;

            std::map<std::string, Parameter<std::string> > stringParameters;
            std::map<std::string, Parameter<std::vector<std::string> > > stringVectorParameters;
            std::map<std::string, Parameter<std::map<std::string, std::string> > > stringMapParameters;

            std::map<std::string, Parameter<float> > floatParameters;
            std::map<std::string, Parameter<std::vector<float> > > floatVectorParameters;

            std::map<std::string, Parameter<bool> > boolParameters;
            std::map<std::string, Parameter<std::vector<bool> > > boolVectorParameters;



            //TODO: Add list and dictionary if needed

            ParameterHandler();
            ParameterHandler(ros::NodeHandle* nodeHandle);

            //Add Parameter methods
            Parameter<XmlRpc::XmlRpcValue> AddParameter(std::string name, std::string description,
                                                        XmlRpc::XmlRpcValue defaultValue);

            Parameter<int> AddParameter(std::string name, std::string description, int defaultValue);
            Parameter<std::vector<int> > AddParameter(std::string name, std::string description,
                                                      std::vector<int> defaultValue);

            Parameter<std::string> AddParameter(std::string name, std::string description, std::string defaultValue);
            Parameter<std::vector<std::string> > AddParameter(std::string name, std::string description,
                                                              std::vector<std::string> defaultValue);
            Parameter<std::map<std::string, std::string> > AddParameter(std::string name, std::string description,
                                                                        std::map<std::string, std::string> defaultValue);

            Parameter<float> AddParameter(std::string name, std::string description, float defaultValue);
            Parameter<std::vector<float> > AddParameter(std::string name, std::string description,
                                                        std::vector<float> defaultValue);

            Parameter<bool> AddParameter(std::string name, std::string description, bool defaultValue);
            Parameter<std::vector<bool> > AddParameter(std::string name, std::string description,
                                                       std::vector<bool> defaultValue);

            Parameter<int> AddParameter(std::string name, std::string subNamespace, std::string description, int defaultValue);
            Parameter<std::vector<int> > AddParameter(std::string name, std::string subNamespace,
                                                      std::string description, std::vector<int> defaultValue);

            Parameter<std::string> AddParameter(std::string name, std::string subNamespace, std::string description,
                                                std::string defaultValue);
            Parameter<std::vector<std::string> > AddParameter(std::string name, std::string subNamespace,
                                                              std::string description, std::vector<std::string> defaultValue);
            Parameter<std::map<std::string, std::string> > AddParameter(std::string name, std::string subNamespace,
                                                                        std::string description,
                                                                        std::map<std::string, std::string> defaultValue);

            Parameter<float> AddParameter(std::string name, std::string subNamespace, std::string description,
                                          float defaultValue);
            Parameter<std::vector<float> > AddParameter(std::string name, std::string subNamespace,
                                                        std::string description, std::vector<float> defaultValue);

            Parameter<bool> AddParameter(std::string name, std::string subNamespace, std::string description,
                                         bool defaultValue);
            Parameter<std::vector<bool> > AddParameter(std::string name, std::string subNamespace,
                                                       std::string description, std::vector<bool> defaultValue);

            //Get Parameter methods
            Parameter<XmlRpc::XmlRpcValue> GetXmlRpcParameter(std::string name);

            Parameter<int> GetIntParameter(std::string name);
            Parameter<std::vector<int> > GetIntVectorParameter(std::string name);

            Parameter<std::string> GetStringParameter(std::string name);            
            Parameter<std::vector<std::string> > GetStringVectorParameter(std::string name);
            Parameter<std::map<std::string, std::string> > GetStringMapParameter(std::string name);

            Parameter<float> GetFloatParameter(std::string name);
            Parameter<std::vector<float> > GetFloatVectorParameter(std::string name);

            Parameter<bool> GetBoolParameter(std::string name);
            Parameter<std::vector<bool> > GetBoolVectorParameter(std::string name);

        private:
            ros::NodeHandle* nodeHandle;

        };
}
