#include "parameterHandler.h"

namespace customparameter
{
    ParameterHandler::ParameterHandler()
    {

    }

    ParameterHandler::ParameterHandler(ros::NodeHandle* nodeHandle) : nodeHandle(nodeHandle)
    {

    }

    //ADD Methods
    /*--------------------------------------------------------------------------------------*/
    void ParameterHandler::AddParameter(std::string name,
                                        std::string  description,
                                        int defaultValue)
    {
        Parameter<int> param(name, description, nodeHandle, defaultValue);
        this->intParameters.insert(std::make_pair(name, param));
    }
    void ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        int defaultValue)
    {
        Parameter<int> param(name, description, subNamespace, nodeHandle, defaultValue);
        this->intParameters.insert(std::make_pair(name, param));
    }

    void ParameterHandler::AddParameter(std::string name,
                                        std::string description,
                                        std::string defaultValue)
    {
        Parameter<std::string> param(name, description, nodeHandle, defaultValue);
        //param.SetDefault();
        this->stringParameters.insert(std::make_pair(name, param));
    }
    void ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        std::string defaultValue)
    {
        Parameter<std::string> param(name, description, subNamespace, nodeHandle, defaultValue);
        //param.SetDefault();
        this->stringParameters.insert(std::make_pair(name, param));
    }

    void ParameterHandler::AddParameter(std::string name,
                                        std::string  description,
                                        float defaultValue)
    {
        Parameter<float> param(name, description, nodeHandle, defaultValue);
        this->floatParameters.insert(std::make_pair(name, param));
    }
    void ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        float defaultValue)
    {
        Parameter<float> param(name, description, subNamespace, nodeHandle, defaultValue);
        this->floatParameters.insert(std::make_pair(name, param));
    }

    void ParameterHandler::AddParameter(std::string name,
                                        std::string  description,
                                        bool defaultValue)
    {
        Parameter<bool> param(name, description, nodeHandle, defaultValue);
        this->boolParameters.insert(std::make_pair(name, param));
    }
    void ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        bool defaultValue)
    {
        Parameter<bool> param(name, description, subNamespace, nodeHandle, defaultValue);
        this->boolParameters.insert(std::make_pair(name, param));
    }

    //Get Methods
    /*--------------------------------------------------------------------------------------*/
    Parameter<int> ParameterHandler::GetIntParameter(std::string name)
    {
        return intParameters[name];
    }

    Parameter<std::string> ParameterHandler::GetStringParameter(std::string name)
    {
        return stringParameters[name];
    }

    Parameter<float> ParameterHandler::GetFloatParameter(std::string name)
    {
        return floatParameters[name];
    }

    Parameter<bool> ParameterHandler::GetBoolParameter(std::string name)
    {
        return boolParameters[name];
    }
}
