#ifndef MRP_HUNGARIAN__VISIBILITY_CONTROL_H_
#define MRP_HUNGARIAN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MRP_HUNGARIAN_EXPORT __attribute__ ((dllexport))
    #define MRP_HUNGARIAN_IMPORT __attribute__ ((dllimport))
  #else
    #define MRP_HUNGARIAN_EXPORT __declspec(dllexport)
    #define MRP_HUNGARIAN_IMPORT __declspec(dllimport)
  #endif
  #ifdef MRP_HUNGARIAN_BUILDING_LIBRARY
    #define MRP_HUNGARIAN_PUBLIC MRP_HUNGARIAN_EXPORT
  #else
    #define MRP_HUNGARIAN_PUBLIC MRP_HUNGARIAN_IMPORT
  #endif
  #define MRP_HUNGARIAN_PUBLIC_TYPE MRP_HUNGARIAN_PUBLIC
  #define MRP_HUNGARIAN_LOCAL
#else
  #define MRP_HUNGARIAN_EXPORT __attribute__ ((visibility("default")))
  #define MRP_HUNGARIAN_IMPORT
  #if __GNUC__ >= 4
    #define MRP_HUNGARIAN_PUBLIC __attribute__ ((visibility("default")))
    #define MRP_HUNGARIAN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MRP_HUNGARIAN_PUBLIC
    #define MRP_HUNGARIAN_LOCAL
  #endif
  #define MRP_HUNGARIAN_PUBLIC_TYPE
#endif

#endif  // MRP_HUNGARIAN__VISIBILITY_CONTROL_H_
