#pragma once

#include <ros/ros.h>
#include <boost/lexical_cast.hpp>

namespace customparameter
{
    template <class T>
    class Parameter
    {
        public:
            std::string name;
            std::string subNamespace = "";
            std::string description;
            ros::NodeHandle* nodeHandle;
            std::string wholeName;

            Parameter();
            Parameter(std::string name, std::string description, ros::NodeHandle* nodeHandle, T defaultValue);
            Parameter(std::string name, std::string subNamespace, std::string description,ros::NodeHandle* nodeHandle, T defaultValue);

            void SetDefault();
            void SetValue(T value);

            T GetValue();

        private:
            T value;
            T defaultValue;

            void CheckInit();
            void CreateWholeName();
    };


    template <class T>
    Parameter<T>::Parameter()
    {


    }

    template <class T>
    Parameter<T>::Parameter(std::string name,   std::string description,
                                                ros::NodeHandle* nodeHandle,
                                                T defaultValue):
        name(name), description(description), nodeHandle(nodeHandle), defaultValue(defaultValue)
    {
        CreateWholeName();
        CheckInit();
    }

    template <class T>
    Parameter<T>::Parameter(std::string name,   std::string description,
                                                std::string subNamespace,
                                                ros::NodeHandle* nodeHandle,
                                                T defaultValue):
        name(name), subNamespace(subNamespace), description(description), nodeHandle(nodeHandle), defaultValue(defaultValue)
    {
        CreateWholeName();
        CheckInit();
    }

    template <class T>
    void Parameter<T>::CreateWholeName()
    {
        wholeName = nodeHandle->getNamespace() + "/" + subNamespace + name;
    }

    template <class T>
    void Parameter<T>::CheckInit()
    {
        if(!nodeHandle->hasParam(wholeName))
        {
            //Set to default
            SetDefault();
        }                            
    }

    template <>
    inline void Parameter<std::vector<std::string> >::SetValue(std::vector<std::string> value)
    {
        this->value = value;

        nodeHandle->setParam(wholeName, value);

        std::string tempString;
        for (int i = 0; i < value.size(); i++)
        {
            tempString += value[i] + " ;";
        }

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to value: " << tempString);
    }

    template <>
    inline void Parameter<std::vector<std::string> >::SetDefault()
    {
        SetValue(this->defaultValue);

        std::string tempString;
        for (int i = 0; i < defaultValue.size(); i++)
        {
            tempString += defaultValue[i] + " ;";
        }

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to default value: " << tempString);
    }

    template <>
    inline void Parameter<std::map<std::string, std::string> >::SetValue(std::map<std::string, std::string> value)
    {
        this->value = value;

        nodeHandle->setParam(wholeName, value);

        std::string tempString;

        for (std::map<std::string, std::string>::iterator it = defaultValue.begin(); it != defaultValue.end(); ++it)
        {
            tempString += it->first + ": " + it->second + ", ";
        }

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to default value: " << tempString);
    }

    template <>
    inline void Parameter<std::map<std::string, std::string> >::SetDefault()
    {
        SetValue(this->defaultValue);

        std::string tempString;

        for (std::map<std::string, std::string>::iterator it = defaultValue.begin(); it != defaultValue.end(); ++it)
        {
            tempString += it->first + ": " + it->second + ", ";
        }

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to default value: " << tempString);
    }

    template <>
    inline void Parameter<std::vector<int> >::SetValue(std::vector<int> value)
    {
        this->value = value;

        nodeHandle->setParam(wholeName, value);

        std::string tempString;
        for (int i = 0; i < value.size(); i++)
        {
            tempString += value[i] + " ;";
        }

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to value: " << tempString);
    }

    template <>
    inline void Parameter<std::vector<int> >::SetDefault()
    {
        SetValue(this->defaultValue);

        std::string tempString;
        for (int i = 0; i < defaultValue.size(); i++)
        {
            tempString += defaultValue[i] + " ;";
        }

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to default value: " << tempString);
    }

    template <>
    inline void Parameter<std::vector<bool> >::SetValue(std::vector<bool> value)
    {
        this->value = value;

        nodeHandle->setParam(wholeName, value);

        std::string tempString;
        for (int i = 0; i < value.size(); i++)
        {
            tempString += boost::lexical_cast<bool>(value[i]) + " ;";
        }

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to value: " << tempString);
    }

    template <>
    inline void Parameter<std::vector<bool> >::SetDefault()
    {
        SetValue(this->defaultValue);

        std::string tempString;
        for (int i = 0; i < defaultValue.size(); i++)
        {
            tempString += boost::lexical_cast<bool>(defaultValue[i]) + " ;";
        }

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to default value: " << tempString);
    }

    template <>
    inline void Parameter<std::vector<float> >::SetValue(std::vector<float> value)
    {
        this->value = value;

        nodeHandle->setParam(wholeName, value);

        std::string tempString;
        for (int i = 0; i < value.size(); i++)
        {
            tempString += boost::lexical_cast<std::string>(value[i]) + " ;";
        }

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to value: " << tempString);
    }

    template <>
    inline void Parameter<std::vector<float> >::SetDefault()
    {
        SetValue(this->defaultValue);

        std::string tempString;
        for (int i = 0; i < defaultValue.size(); i++)
        {
            tempString += boost::lexical_cast<std::string>(defaultValue[i]) + " ;";
        }

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to default value: " << tempString);
    }

    template <class T>
    void Parameter<T>::SetDefault()
    {
        SetValue(this->defaultValue);

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to default value: " << defaultValue);
    }

    template <class T>
    void Parameter<T>::SetValue(T value)
    {
        this->value = value;

        nodeHandle->setParam(wholeName, value);

        ROS_INFO_STREAM("Paramter: " << wholeName << " was set to value: " << value);
    }

    template <class T>
    T Parameter<T>::GetValue()
    {
        T value;
        if(wholeName.length() > 0)
        {
            nodeHandle->getParam(wholeName, value);

            this->value = value;

            return value;
        }
        else
        {
            return value;
        }
    }
}
