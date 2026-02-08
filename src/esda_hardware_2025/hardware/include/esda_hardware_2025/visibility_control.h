#ifndef ESDA_HARDWARE_2025__VISIBILITY_CONTROL_H_
#define ESDA_HARDWARE_2025__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ESDA_HARDWARE_2025_EXPORT __attribute__ ((dllexport))
    #define ESDA_HARDWARE_2025_IMPORT __attribute__ ((dllimport))
  #else
    #define ESDA_HARDWARE_2025_EXPORT __declspec(dllexport)
    #define ESDA_HARDWARE_2025_IMPORT __declspec(dllimport)
  #endif

  #ifdef ESDA_HARDWARE_2025_BUILDING_DLL
    #define ESDA_HARDWARE_2025_PUBLIC ESDA_HARDWARE_2025_EXPORT
  #else
    #define ESDA_HARDWARE_2025_PUBLIC ESDA_HARDWARE_2025_IMPORT
  #endif

  #define ESDA_HARDWARE_2025_PUBLIC_TYPE ESDA_HARDWARE_2025_PUBLIC
  #define ESDA_HARDWARE_2025_LOCAL

#else
  #define ESDA_HARDWARE_2025_EXPORT __attribute__ ((visibility("default")))
  #define ESDA_HARDWARE_2025_IMPORT

  #if __GNUC__ >= 4
    #define ESDA_HARDWARE_2025_PUBLIC __attribute__ ((visibility("default")))
    #define ESDA_HARDWARE_2025_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ESDA_HARDWARE_2025_PUBLIC
    #define ESDA_HARDWARE_2025_LOCAL
  #endif

  #define ESDA_HARDWARE_2025_PUBLIC_TYPE ESDA_HARDWARE_2025_PUBLIC
#endif

#endif  // ESDA_HARDWARE_2025__VISIBILITY_CONTROL_H_
