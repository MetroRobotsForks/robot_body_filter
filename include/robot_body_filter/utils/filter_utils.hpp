#ifndef ROBOT_BODY_FILTER_UTILS_FILTER_UTILS_HPP
#define ROBOT_BODY_FILTER_UTILS_FILTER_UTILS_HPP

#include <map>

#include <filters/filter_base.hpp>

#include "robot_body_filter/utils/string_utils.hpp"

namespace robot_body_filter
{

template<typename F>
class FilterBase : public filters::FilterBase<F>
{

protected:

  /** \brief Type of function that converts anything to a string. */
  template <typename T> using ToStringFn = std::string (*)(const T&);

  /**
   * \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \tparam T Param type.
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
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
             ToStringFn<T> valueToStringFn = &to_string)
  {
    T value;
    if (this->params_interface_->has_parameter(name) && filters::FilterBase<F>::getParam(name, value))
    {
      if (valueToStringFn != nullptr)
      {
        RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(), this->getName() << ": Found parameter: " << name <<
                                        ", value: " << valueToStringFn(value) <<
                                        prependIfNonEmpty(unit, " "));
      }
      if (defaultUsed != nullptr)
        *defaultUsed = false;
      return value;
    }

    if (valueToStringFn != nullptr)
    {
      RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(), this->getName() << ": Parameter " << name
                                      << " not defined, assigning default: "
                                      << valueToStringFn(defaultValue)
                                      << prependIfNonEmpty(unit, " "));
    }
    if (defaultUsed != nullptr)
      *defaultUsed = true;
    return defaultValue;
  }

  /** \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   */
  std::string getParamVerbose(const std::string &name, const char* defaultValue,
                              const std::string &unit = "", bool* defaultUsed = nullptr,
                              ToStringFn<std::string> valueToStringFn = &to_string)
  {
    return this->getParamVerbose(name, std::string(defaultValue), unit, defaultUsed, valueToStringFn);
  }



  // getParam specializations for unsigned values

  /** \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   * \throw std::invalid_argument If the loaded value is negative.
   */
  uint64_t getParamVerbose(const std::string &name, const uint64_t &defaultValue,
                           const std::string &unit = "", bool* defaultUsed = nullptr,
                           ToStringFn<int> valueToStringFn = &to_string)
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
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
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
                               ToStringFn<int> valueToStringFn = &to_string)
  {
    return this->getParamUnsigned<unsigned int, int>(name, defaultValue, unit, defaultUsed,
        valueToStringFn);
  }

  // ROS types specializations

  /** \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   */
  rclcpp::Duration getParamDuration(const std::string &name,
                                const rclcpp::Duration &defaultValue,
                                const std::string &unit = "",
                                bool* defaultUsed = nullptr,
                                ToStringFn<double> valueToStringFn = &to_string)
  {
    double temp_value = getParamVerbose(name, defaultValue.seconds(), unit, defaultUsed, valueToStringFn);
    return rclcpp::Duration::from_seconds(temp_value);
  }

  /** \brief Get the value of the given filter parameter as a set of strings, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \tparam T Type of the values in the set. Only std::string and double are supported.
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
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
      ToStringFn<std::vector<T>> valueToStringFn = &to_string)
  {
    std::vector<T> vector(defaultValue.begin(), defaultValue.end());
    vector = this->getParamVerbose(name, vector, unit, defaultUsed, valueToStringFn);
    return std::set<T>(vector.begin(), vector.end());
  }

  template<typename T, typename MapType=std::map<std::string, T>>
  MapType getParamVerboseMap(
      const std::string &name,
      const std::map<std::string, T> &defaultValue = std::map<std::string, T>(),
      const std::string &unit = "",
      bool* defaultUsed = nullptr,
      ToStringFn<MapType> valueToStringFn = &to_string)
  {
    MapType value;

    std::string prefix = this->param_prefix_ + name;
    auto parameter_value_map = this->params_interface_->get_parameter_overrides();
    for (auto& pairParam : parameter_value_map)
    {
      if (!startsWith(pairParam.first, prefix)) continue;
      std::string sub_name = pairParam.first.substr(prefix.length() + 1);
      auto v = pairParam.second.template get<T>();
      value[sub_name] = v;
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
                                          << prependIfNonEmpty(unit, " "));
      }
    }
    else
    {
      RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(), this->getName() << ": Found parameter: " << name <<
                                                                 ", value: " << valueToStringFn(value) <<
                                                                 prependIfNonEmpty(unit, " "));
    }

    return value;
  }

private:

  template<typename Result, typename Param>
  Result getParamUnsigned(const std::string &name, const Result &defaultValue,
                          const std::string &unit = "", bool* defaultUsed = nullptr,
                          ToStringFn<Param> valueToStringFn = &to_string)
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
                      ToStringFn<Param> valueToStringFn = &to_string)
  {
    const Param paramValue = this->getParamVerbose(name, defaultValue, unit, defaultUsed,
        valueToStringFn);
    return Result(paramValue);
  }

};

}
#endif //ROBOT_BODY_FILTER_UTILS_FILTER_UTILS_HPP
