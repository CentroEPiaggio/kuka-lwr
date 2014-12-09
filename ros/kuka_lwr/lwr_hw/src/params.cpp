#include <param.h>

namespace terse_roscpp {
  namespace param {

    bool size(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        size_t &size,
        const std::string &description,
        const bool required)
    {
      // Get the parameter
      XmlRpc::XmlRpcValue xml_container;
      if( !get(nh, param_name, xml_container, description, required) ) {
        return false;
      }

      // Make sure it's an array type
      if(xml_container.getType() != XmlRpc::XmlRpcValue::TypeArray 
          && xml_container.getType() != XmlRpc::XmlRpcValue::TypeStruct
          && xml_container.getType() != XmlRpc::XmlRpcValue::TypeString) 
      {
        std::ostringstream oss;
        oss<<"Requested parameter is not a container!"
          <<" Namespace: "<<nh.getNamespace()
          <<" Parameter: "<<param_name
          <<" Type: "<<get_xml_rpc_type_name(xml_container.getType());
        throw ros::InvalidParameterException(oss.str());
      }

      // Resize the target vector
      size = xml_container.size();

      return true;
    }

    bool children(
        const ros::NodeHandle &nh,
        const std::string &param_name,
        std::vector<std::string> &children,
        const std::string &description,
        const bool required)
    {
      // Get the parameter
      XmlRpc::XmlRpcValue xml_array;
      if( !get(nh, param_name, xml_array, description, required) ) {
        return false;
      }

      // Make sure it's an array type
      if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        std::ostringstream oss;
        oss<<"Requested parameter is not a struct!"
          <<" Namespace: "<<nh.getNamespace()
          <<" Parameter: "<<param_name
          <<" Type: "<<get_xml_rpc_type_name(xml_array.getType());
        throw ros::InvalidParameterException(oss.str());
      }

      // Resize the target vector
      TerseXmlRpcValue terse_xml_array(xml_array);
      XmlRpc::XmlRpcValue::ValueStruct xml_map = terse_xml_array;
      children.resize(0);

      for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it=xml_map.begin();
          it != xml_map.end();
          ++it)
      {
        children.push_back(it->first);
      }

      return true;
    }
  }
}