#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define JVRC1Controller_DLLIMPORT __declspec(dllimport)
#  define JVRC1Controller_DLLEXPORT __declspec(dllexport)
#  define JVRC1Controller_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define JVRC1Controller_DLLIMPORT __attribute__((visibility("default")))
#    define JVRC1Controller_DLLEXPORT __attribute__((visibility("default")))
#    define JVRC1Controller_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define JVRC1Controller_DLLIMPORT
#    define JVRC1Controller_DLLEXPORT
#    define JVRC1Controller_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef JVRC1Controller_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define JVRC1Controller_DLLAPI
#  define JVRC1Controller_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef JVRC1Controller_EXPORTS
#    define JVRC1Controller_DLLAPI JVRC1Controller_DLLEXPORT
#  else
#    define JVRC1Controller_DLLAPI JVRC1Controller_DLLIMPORT
#  endif // JVRC1Controller_EXPORTS
#  define JVRC1Controller_LOCAL JVRC1Controller_DLLLOCAL
#endif // JVRC1Controller_STATIC