#ifndef PLUGINLIB__CLASS_LIST_MACROS_HPP_
#define PLUGINLIB__CLASS_LIST_MACROS_HPP_

#include <class_loader/class_loader.hpp>

/// Register a class with class loader to effectively export it for plugin loading later.
/**
 * \def PLUGINLIB_EXPORT_CLASS(class_type, base_class_type)
 * \param class_type The real class name with namespace qualifier (e.g. Animals::Lion)
 * \param base_class_type The real base class type from which class_type inherits
 */
#define PLUGINLIB_EXPORT_CLASS(class_type, base_class_type) \
  CLASS_LOADER_REGISTER_CLASS(class_type, base_class_type)

#endif  // PLUGINLIB__CLASS_LIST_MACROS_HPP_
