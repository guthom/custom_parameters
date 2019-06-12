#include "custom_parameter/parameterHandler.h"

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
    Parameter<XmlRpc::XmlRpcValue> ParameterHandler::AddParameter(std::string name,
                                                                  std::string  description,
                                                                  XmlRpc::XmlRpcValue defaultValue)
    {
        Parameter<XmlRpc::XmlRpcValue> param(name, description, nodeHandle, defaultValue);
        this->xmlRpcParameters.insert(std::make_pair(name, param));
        return xmlRpcParameters[name];
    }

    Parameter<int> ParameterHandler::AddParameter(std::string name,
                                        std::string  description,
                                        int defaultValue)
    {
        Parameter<int> param(name, description, nodeHandle, defaultValue);
        this->intParameters.insert(std::make_pair(name, param));
        return intParameters[name];
    }
    Parameter<int> ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        int defaultValue)
    {
        Parameter<int> param(name, description, subNamespace, nodeHandle, defaultValue);
        this->intParameters.insert(std::make_pair(name, param));
        return intParameters[name];

    }

    Parameter<std::string> ParameterHandler::AddParameter(std::string name,
                                        std::string description,
                                        std::string defaultValue)
    {
        Parameter<std::string> param(name, description, nodeHandle, defaultValue);
        //param.SetDefault();
        this->stringParameters.insert(std::make_pair(name, param));
        return stringParameters[name];
    }

    Parameter<std::map<std::string, std::string> > ParameterHandler::AddParameter(std::string name,
                                                                                  std::string description,
                                                                                  std::map<std::string,
                                                                                          std::string> defaultValue)
    {
        Parameter<std::map<std::string, std::string> > param(name, description, nodeHandle, defaultValue);
        //param.SetDefault();
        this->stringMapParameters.insert(std::make_pair(name, param));
        return stringMapParameters[name];
    };

    Parameter<std::string> ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        std::string defaultValue)
    {
        Parameter<std::string> param(name, description, subNamespace, nodeHandle, defaultValue);
        //param.SetDefault();
        this->stringParameters.insert(std::make_pair(name, param));
        return stringParameters[name];
    }

    Parameter<std::vector<std::string> > ParameterHandler::AddParameter(std::string name,
                                        std::string description,
                                        std::vector<std::string> defaultValue)
    {
        Parameter<std::vector<std::string> > param(name, description, nodeHandle, defaultValue);
        //param.SetDefault();
        this->stringVectorParameters.insert(std::make_pair(name, param));
        return stringVectorParameters[name];
    }
    Parameter<std::vector<std::string> > ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        std::vector<std::string> defaultValue)
    {
        Parameter<std::vector<std::string> > param(name, description, subNamespace, nodeHandle, defaultValue);
        //param.SetDefault();
        this->stringVectorParameters.insert(std::make_pair(name, param));
        return stringVectorParameters[name];
    }

    Parameter<std::map<std::string, std::string> > ParameterHandler::AddParameter(std::string name,
                                                                                  std::string subNamespace,
                                                                                  std::string description,
                                                                                  std::map<std::string,
                                                                                          std::string> defaultValue)
    {
        Parameter<std::map<std::string, std::string> > param(name, description, subNamespace, nodeHandle, defaultValue);
        //param.SetDefault();
        this->stringMapParameters.insert(std::make_pair(name, param));
        return stringMapParameters[name];
    };


    Parameter<std::vector<int> >  ParameterHandler::AddParameter(std::string name,
                                        std::string description,
                                        std::vector<int> defaultValue)
    {
        Parameter<std::vector<int> > param(name, description, nodeHandle, defaultValue);
        //param.SetDefault();
        this->intVectorParameters.insert(std::make_pair(name, param));
        return intVectorParameters[name];
    }
    Parameter<std::vector<int> >  ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        std::vector<int> defaultValue)
    {
        Parameter<std::vector<int> > param(name, description, subNamespace, nodeHandle, defaultValue);
        //param.SetDefault();
        this->intVectorParameters.insert(std::make_pair(name, param));
        return intVectorParameters[name];
    }

    Parameter<std::vector<float> > ParameterHandler::AddParameter(std::string name,
                                        std::string description,
                                        std::vector<float> defaultValue)
    {
        Parameter<std::vector<float> > param(name, description, nodeHandle, defaultValue);
        //param.SetDefault();
        this->floatVectorParameters.insert(std::make_pair(name, param));
        return floatVectorParameters[name];
    }
    Parameter<std::vector<float> > ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        std::vector<float> defaultValue)
    {
        Parameter<std::vector<float> > param(name, description, subNamespace, nodeHandle, defaultValue);
        //param.SetDefault();
        this->floatVectorParameters.insert(std::make_pair(name, param));
        return floatVectorParameters[name];
    }

    Parameter<std::vector<bool> > ParameterHandler::AddParameter(std::string name,
                                        std::string description,
                                        std::vector<bool> defaultValue)
    {
        Parameter<std::vector<bool> > param(name, description, nodeHandle, defaultValue);
        //param.SetDefault();
        this->boolVectorParameters.insert(std::make_pair(name, param));
        return boolVectorParameters[name];
    }
    Parameter<std::vector<bool> > ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        std::vector<bool> defaultValue)
    {
        Parameter<std::vector<bool> > param(name, description, subNamespace, nodeHandle, defaultValue);
        //param.SetDefault();
        this->boolVectorParameters.insert(std::make_pair(name, param));
        return boolVectorParameters[name];
    }

    Parameter<float> ParameterHandler::AddParameter(std::string name,
                                        std::string  description,
                                        float defaultValue)
    {
        Parameter<float> param(name, description, nodeHandle, defaultValue);
        this->floatParameters.insert(std::make_pair(name, param));
        return floatParameters[name];
    }
    Parameter<float> ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        float defaultValue)
    {
        Parameter<float> param(name, description, subNamespace, nodeHandle, defaultValue);
        this->floatParameters.insert(std::make_pair(name, param));
        return floatParameters[name];
    }

    Parameter<bool> ParameterHandler::AddParameter(std::string name,
                                        std::string  description,
                                        bool defaultValue)
    {
        Parameter<bool> param(name, description, nodeHandle, defaultValue);
        this->boolParameters.insert(std::make_pair(name, param));
        return boolParameters[name];
    }
    Parameter<bool> ParameterHandler::AddParameter(std::string name,
                                        std::string subNamespace,
                                        std::string description,
                                        bool defaultValue)
    {
        Parameter<bool> param(name, description, subNamespace, nodeHandle, defaultValue);
        this->boolParameters.insert(std::make_pair(name, param));
        return boolParameters[name];
    }

    //Get Methods
    /*--------------------------------------------------------------------------------------*/
    Parameter<XmlRpc::XmlRpcValue> ParameterHandler::GetXmlRpcParameter(std::string name)
    {
        return xmlRpcParameters[name];
    }

    Parameter<int> ParameterHandler::GetIntParameter(std::string name)
    {
        return intParameters[name];
    }

    Parameter<std::string> ParameterHandler::GetStringParameter(std::string name)
    {
        return stringParameters[name];
    }

    Parameter<std::vector<std::string> > ParameterHandler::GetStringVectorParameter(std::string name)
    {
        return stringVectorParameters[name];
    }

    Parameter<std::map<std::string, std::string> > ParameterHandler::GetStringMapParameter(std::string name)
    {
        return stringMapParameters[name];
    };

    Parameter<std::vector<float> > ParameterHandler::GetFloatVectorParameter(std::string name)
    {
        return floatVectorParameters[name];
    }

    Parameter<std::vector<int> > ParameterHandler::GetIntVectorParameter(std::string name)
    {
        return intVectorParameters[name];
    }

    Parameter<std::vector<bool> > ParameterHandler::GetBoolVectorParameter(std::string name)
    {
        return boolVectorParameters[name];
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
