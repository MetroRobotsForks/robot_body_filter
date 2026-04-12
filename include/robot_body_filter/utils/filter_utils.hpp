#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <map>

#include <cras_cpp_common/string_utils.hpp>
#include <filters/filter_base.hpp>

namespace robot_body_filter
{

template<typename F>
class FilterBase : public filters::FilterBase<F>
{

protected:

  /** \brief Type of function that converts anything to a string. */
  template <typename T> using ToStringFn = std::string (*)(const T&);

  bool hasParam(const std::string& name) const
  {
    const auto paramName = this->param_prefix_ + name;
    if (this->params_interface_->has_parameter(paramName))
      return true;
    const auto overrides = this->params_interface_->get_parameter_overrides();
    return overrides.count(paramName) > 0;
  }

  /**
   * \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \tparam T Param type.
   * \param name Name of the parameter (without the `filterN.params.` prefix).
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \param defaultUsed Whether the default value was used.
   * \param valueToStringFn Function that converts valid/default values to string (for console
   *             logging). Set to nullptr to disable logging.
   * \return The loaded param value.
   */
  template<typename T>
  T getParamVerbose(const std::string &name, const T &defaultValue = T(),
             const std::string &unit = "", bool* defaultUsed = nullptr,
             ToStringFn<T> valueToStringFn = &cras::to_string)
  {
    T value;
    try
    {
      if (this->hasParam(name) && filters::FilterBase<F>::getParam(name, value))
      {
        if (valueToStringFn != nullptr)
        {
          RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(), this->getName() << ": Found parameter: " << name <<
                                          ", value: " << valueToStringFn(value) <<
                                          cras::prependIfNonEmpty(unit, " "));
        }
        if (defaultUsed != nullptr)
          *defaultUsed = false;
        return value;
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(this->logging_interface_->get_logger(), this->getName() << ": Error getting value of parameter "
        << name << ": " << e.what());
    }

    if (valueToStringFn != nullptr)
    {
      RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(), this->getName() << ": Parameter " << name
                                      << " not defined, assigning default: "
                                      << valueToStringFn(defaultValue)
                                      << cras::prependIfNonEmpty(unit, " "));
    }
    if (defaultUsed != nullptr)
      *defaultUsed = true;
    return defaultValue;
  }

  /** \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \param name Name of the parameter (without the `filterN.params.` prefix).
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   */
  std::string getParamVerbose(const std::string &name, const char* defaultValue,
                              const std::string &unit = "", bool* defaultUsed = nullptr,
                              ToStringFn<std::string> valueToStringFn = &cras::to_string)
  {
    return this->getParamVerbose(name, std::string(defaultValue), unit, defaultUsed, valueToStringFn);
  }



  // getParam specializations for unsigned values

  /** \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \param name Name of the parameter (without the `filterN.params.` prefix).
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   * \throw std::invalid_argument If the loaded value is negative.
   */
  uint64_t getParamVerbose(const std::string &name, const uint64_t &defaultValue,
                           const std::string &unit = "", bool* defaultUsed = nullptr,
                           ToStringFn<int> valueToStringFn = &cras::to_string)
  {
    return this->getParamUnsigned<uint64_t, int>(name, defaultValue, unit, defaultUsed,
        valueToStringFn);
  }

  // there actually is an unsigned int implementation of FilterBase::getParam,
  // but it doesn't tell you when the passed value is negative - instead it just
  // returns false
  /** \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \param name Name of the parameter (without the `filterN.params.` prefix).
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   * \throw std::invalid_argument If the loaded value is negative.
   */
  unsigned int getParamVerbose(const std::string &name,
                               const unsigned int &defaultValue,
                               const std::string &unit = "",
                               bool* defaultUsed = nullptr,
                               ToStringFn<int> valueToStringFn = &cras::to_string)
  {
    return this->getParamUnsigned<unsigned int, int>(name, defaultValue, unit, defaultUsed,
        valueToStringFn);
  }

  // ROS types specializations

  /** \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \param name Name of the parameter (without the `filterN.params.` prefix).
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   */
  rclcpp::Duration getParamDuration(const std::string &name,
                                const rclcpp::Duration &defaultValue,
                                const std::string &unit = "",
                                bool* defaultUsed = nullptr,
                                ToStringFn<double> valueToStringFn = &cras::to_string)
  {
    double temp_value = getParamVerbose(name, defaultValue.seconds(), unit, defaultUsed, valueToStringFn);
    return rclcpp::Duration::from_seconds(temp_value);
  }

  /** \brief Get the value of the given filter parameter as a set of strings, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \tparam T Type of the values in the set. Only std::string and double are supported.
   * \param name Name of the parameter (without the `filterN.params.` prefix).
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   */
  template<typename T>
  std::set<T> getParamVerboseSet(
      const std::string &name,
      const std::set<T> &defaultValue = std::set<T>(),
      const std::string &unit = "",
      bool* defaultUsed = nullptr,
      ToStringFn<std::vector<T>> valueToStringFn = &cras::to_string)
  {
    std::vector<T> vector(defaultValue.begin(), defaultValue.end());
    vector = this->getParamVerbose(name, vector, unit, defaultUsed, valueToStringFn);
    return std::set<T>(vector.begin(), vector.end());
  }

   /** \brief Get the value of the given filter parameter as a map with string keys, falling back to the
    *        specified default value, and print out a ROS info/warning message with
    *        the loaded values.
    * \tparam T Type of the values in the map.
    * \tparam MapType Type of the map. Only maps with string keys are expected to be used.
    * \param name Name of the parameter (without the `filterN.params.` prefix).
    * \param defaultValue The default value to use.
    * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
    *             messages more informative.
    * \return The loaded param value.
    */
  template<typename T, typename MapType=std::map<std::string, T>>
  MapType getParamVerboseMap(
      const std::string &name,
      const std::map<std::string, T> &defaultValue = std::map<std::string, T>(),
      const std::string &unit = "",
      bool* defaultUsed = nullptr,
      ToStringFn<MapType> valueToStringFn = &cras::to_string)
  {
    MapType value;

    std::string prefix = this->param_prefix_ + name + ".";
    auto parameter_value_map = this->params_interface_->get_parameter_overrides();
    for (const auto& [paramName, paramValue] : parameter_value_map)
    {
      if (!cras::startsWith(paramName, prefix))
        continue;
      std::string sub_name = paramName.substr(prefix.length());
      try
      {
        value[sub_name] = paramValue.template get<T>();
      }
      catch (const std::exception& e)
      {
        if constexpr (std::is_same_v<double, T>)
        {
          try
          {
            value[sub_name] = static_cast<double>(paramValue.template get<int>());
          }
          catch (const std::exception&)
          {
            RCLCPP_ERROR_STREAM(this->logging_interface_->get_logger(), this->getName() << ": Error getting value of parameter "
            << paramName << ": " << e.what());
          }
        }
        else
        {
          RCLCPP_ERROR_STREAM(this->logging_interface_->get_logger(), this->getName() << ": Error getting value of parameter "
            << paramName << ": " << e.what());
        }
      }
    }

    if (value.empty())
    {
      value = defaultValue;
      if (defaultUsed != nullptr)
        *defaultUsed = true;
      if (valueToStringFn != nullptr)
      {
          RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(), this->getName() << ": Parameter " << name
                                          << " not defined, assigning default: "
                                          << valueToStringFn(defaultValue)
                                          << cras::prependIfNonEmpty(unit, " "));
      }
    }
    else
    {
      if (defaultUsed != nullptr)
        *defaultUsed = false;
      RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(), this->getName() << ": Found parameter: " << name <<
                                                                 ", value: " << valueToStringFn(value) <<
                                                                 cras::prependIfNonEmpty(unit, " "));
    }

    return value;
  }

private:

  template<typename Result, typename Param>
  Result getParamUnsigned(const std::string &name, const Result &defaultValue,
                          const std::string &unit = "", bool* defaultUsed = nullptr,
                          ToStringFn<Param> valueToStringFn = &cras::to_string)
  {
    const Param signedValue = this->getParamVerbose(name, static_cast<Param>(defaultValue), unit,
        defaultUsed, valueToStringFn);
    if (signedValue < 0)
    {
      if (valueToStringFn != nullptr)
      {
        RCLCPP_ERROR_STREAM(this->logging_interface_->get_logger(), this->getName() << ": Value " << valueToStringFn(signedValue) <<
                                         " of unsigned parameter " << name << " is negative.");
      }
      throw std::invalid_argument(name);
    }
    return static_cast<Result>(signedValue);
  }

  // generic casting getParam()
  template<typename Result, typename Param>
  Result getParamCast(const std::string &name, const Param &defaultValue,
                      const std::string &unit = "", bool* defaultUsed = nullptr,
                      ToStringFn<Param> valueToStringFn = &cras::to_string)
  {
    const Param paramValue = this->getParamVerbose(name, defaultValue, unit, defaultUsed,
        valueToStringFn);
    return Result(paramValue);
  }

};

}
