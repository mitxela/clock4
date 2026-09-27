// Firmware version + a per-build id. The pre-build step (gen-build-id.sh, wired via the CubeIDE
// prebuildStep in .cproject) regenerates build_id.h with a fresh epoch-seconds token AND deletes this
// TU's object so __DATE__/__TIME__ and BUILD_ID are re-stamped on EVERY build. That makes every image
// byte-unique, so the bootloader's CRC compare always reflashes and the reported version string tells
// same-day builds apart (was: two 0.0.5 builds on one day could be byte-identical -> silently skipped).
// build_id.h is a generated artifact (gitignored); the __has_include guard keeps a bare checkout or the
// emulator build (which never compiles this TU) compiling with a "dev" fallback.
#if defined(__has_include)
#  if __has_include("build_id.h")
#    include "build_id.h"
#  endif
#endif
#ifndef BUILD_ID
#  define BUILD_ID "dev"
#endif

#define VERSION_STRING "Version 0.0.5+" BUILD_ID " "
#include "../../../version.h"
