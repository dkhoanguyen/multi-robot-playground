#ifndef MRP_LOCAL_SERVER_CORE_LOCAL_CONTROLLER_SERVER_HPP_
#define MRP_LOCAL_SERVER_CORE_LOCAL_CONTROLLER_SERVER_HPP_

namespace local_server_core
{
  class LocalControllerCore
  {
  public:
    virtual ~LocalControllerCore(){};
    virtual void initialise() = 0;
  };
} // namespace name

#endif