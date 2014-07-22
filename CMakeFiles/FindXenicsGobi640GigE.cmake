# - Try to find Xenics Gobi640GigE SDK library installation
# the library for Linux 64-bit can be downloaded from www.xenics.com/files/support/Linux_SDK_25.zip
#
# The following variables are searched for defaults
#  XenicsGobi640GigE_DIR:         Base directory of Xenics Gobi640GigE SDK tree to use.
#  Note that Xeneth SDK version 2.5 (Linux 64-bit release 409) provides only the dynamic library, which is placed in /usr/lib64. The headers are placed in /usr/share/xeneth
#
# The following are set after configuration is done:
#  XenicsGobi640GigE_FOUND
#  XenicsGobi640GigE_INCLUDE_DIR
#  XenicsGobi640GigE_LINK_DIRECTORIES
#
SET( XenicsGobi640GigE_FOUND 0 )

################################
# Find the Include Directory

FIND_PATH (XenicsGobi640GigE_INCLUDE_DIR
  NAMES
  "XCamera.h" # on Linux 64-bit
  PATHS
  $ENV{XenicsGobi640GigE_DIR}
  PATH_SUFFIXES
  Include # on Linux 64-bit
)

################################
# Find the Library Directory

# - /usr/lib64 on Linux 64-bit
FIND_PATH (XenicsGobi640GigE_LINK_DIRECTORIES
  NAMES
  # Windows SDK (prefer static lib)
  
  # Linux/QNX/Mac SDKs (prefer dynamic if available)
  "libxeneth.so" # Shared Object in Linux SDKs
  PATHS
  PATH_SUFFIXES
)

IF( EXISTS ${XenicsGobi640GigE_INCLUDE_DIR} AND EXISTS ${XenicsGobi640GigE_LINK_DIRECTORIES} )
  SET( XenicsGobi640GigE_FOUND 1 )
ENDIF( EXISTS ${XenicsGobi640GigE_INCLUDE_DIR} AND EXISTS ${XenicsGobi640GigE_LINK_DIRECTORIES} )

