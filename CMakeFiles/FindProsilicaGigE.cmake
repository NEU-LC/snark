# - Try to find Prosilica GigE SDK library installation
# See http://www.prosilica.com/support/gige/ge_starter.html
#
# The following variables are searched for defaults
#  ProsilicaGigE_DIR:         Base directory of GigE SDK tree to use.
#  ProsilicaGigE_LIB_SUFFIX:  Suffix to select your lib manually
#    Since the SDKs contain a few different libs for different
#    architectures/gcc versions/linking styles, it may be useful
#    to be able to select one specifically rather than relying
#    on this script to automatically pick the right one
#    e.g. export ProsilicaGigE_LIB_SUFFIX='bin-pc/x86' for dynamic lib on x86 (use this for Linux)
#                                        ='lib-pc/x86/4.2.4' static lib w/ gcc 4.2.4 (use this for QNX 6.4)
#                                        ='lib-pc/arm/3.3' for static lib on arm w/ gcc 3.3
#
#    Alternatively, you can remove the libs you don't want to link to, or copy
#    your selected lib to the base directory of the SDK
#
# The following are set after configuration is done:
#  ProsilicaGigE_FOUND
#  ProsilicaGigE_INCLUDE_DIR
#  ProsilicaGigE_LINK_DIRECTORIES
#
SET( ProsilicaGigE_FOUND 0 )

################################
# Find the Include Directory

FIND_PATH (ProsilicaGigE_INCLUDE_DIR
  NAMES
  "PvApi.h" # Same name on all OSes
  PATHS
  $ENV{ProsilicaGigE_DIR}
  PATH_SUFFIXES
  inc-pc # Same sub-directory on all OSes
)

################################
# Find the Library Directory

# The library files to link to can be found in a few places...
# - GigESDK/lib-pc on Windows (or lib-pc/x64 for Windows 64-bit)
# - GigESDK/lib-pc/architecture/gcc-version/ for POSIX/gcc static linking
# - GigESDK/bin-pc/architecture/ for POSIX/gcc dynamic linking
# This will attempt to find the 'best' one but this is based on guess-work so it might be better to pick your own link directory manually with the PROSILICA_GigE_LIB_SUFFIX variable
FIND_PATH (ProsilicaGigE_LINK_DIRECTORIES
  NAMES
  # Windows SDK (prefer static lib)
  "PvAPI.lib" # Static Library in Windows SDK
  "PvAPI.dll" # Dynamic Link Library in Windows SDK
  # Linux/QNX/Mac SDKs (prefer dynamic if available)
  "libPvAPI.so" # Shared Object in Linux / QNX SDKs
  "libPvAPI.dylib" # Dynamic Library in Mac OS X SDK
  "libPvAPI.a" # Static Library in Linux, QNX and Mac OS X SDKs
  PATHS
  $ENV{ProsilicaGigE_DIR}
  PATH_SUFFIXES
  # First check if the user has specified one manually before checking all the default suffixes
  $ENV{ProsilicaGigE_LIB_SUFFIX}
  lib-pc
  bin-pc
  # Dynamic Library Locations (prefer x86 architecture since most of us use that...)
  bin-pc/x86
  bin-pc/x64
  bin-pc/ppc
  bin-pc/arm
  # Static Library Locations (prefer newest version of GCC on x86)
  lib-pc/x86
  lib-pc/x86/4.3
  lib-pc/x86/4.2.4
  lib-pc/x86/4.2
  lib-pc/x86/4.1
  lib-pc/x86/4.0
  lib-pc/x86/3.4
  lib-pc/x86/3.3.5
  lib-pc/x86/3.3
  lib-pc/x86/2.95.3
  lib-pc/x64
  lib-pc/x64/4.3
  lib-pc/x64/4.2
  lib-pc/x64/4.1
  lib-pc/x64/4.0
  lib-pc/ppc
  lib-pc/ppc/4.3
  lib-pc/ppc/4.2
  lib-pc/ppc/4.1
  lib-pc/ppc/4.0
  lib-pc/ppc/3.4
  lib-pc/ppc/3.3
  lib-pc/arm
  lib-pc/arm/3.3
)

IF( EXISTS ${ProsilicaGigE_INCLUDE_DIR} AND EXISTS ${ProsilicaGigE_LINK_DIRECTORIES} )
  SET( ProsilicaGigE_FOUND 1 )
ENDIF( EXISTS ${ProsilicaGigE_INCLUDE_DIR} AND EXISTS ${ProsilicaGigE_LINK_DIRECTORIES} )

