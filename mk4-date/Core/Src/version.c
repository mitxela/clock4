// Date-board firmware version + a per-build id, matching the time board (mk4-time). The pre-build step
// (gen-build-id.sh, wired via the CubeIDE prebuildStep in .cproject) regenerates build_id.h with a
// fresh epoch-seconds token AND deletes this TU's object so __DATE__/__TIME__ and BUILD_ID are
// re-stamped on EVERY build. That makes every fwd.bin byte-unique, so the time board's CRC compare
// always reflashes it and the reported version tells same-day builds apart. build_id.h is a generated
// artifact (gitignored); the __has_include guard keeps a bare checkout compiling with a "dev" fallback.
#if defined(__has_include)
#  if __has_include("build_id.h")
#    include "build_id.h"
#  endif
#endif
#ifndef BUILD_ID
#  define BUILD_ID "dev"
#endif

#define VERSION_STRING "Version 0.0.2+" BUILD_ID " "
#include "../../../version.h"
