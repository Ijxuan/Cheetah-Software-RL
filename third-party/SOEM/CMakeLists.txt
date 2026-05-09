set(EC_BUFSIZE EC_MAXECATFRAME CACHE STRING "standard frame buffer size in bytes")
set(EC_MAXBUF 16 CACHE STRING "number of frame buffers per channel (tx, rx1 rx2)")
set(EC_MAXEEPBITMAP 128 CACHE STRING "size of EEPROM bitmap cache")
set(EC_MAXEEPBUF "EC_MAXEEPBITMAP << 5" CACHE STRING "size of EEPROM cache buffer")
set(EC_LOGGROUPOFFSET 16 CACHE STRING "default group size in 2^x")
set(EC_MAXELIST 64 CACHE STRING "max. entries in EtherCAT error list")
set(EC_MAXNAME 40 CACHE STRING "max. length of readable name in slavelist and Object Description List")
set(EC_MAXSLAVE 200 CACHE STRING "max. number of slaves in array")
set(EC_MAXGROUP 2 CACHE STRING "max. number of groups")
set(EC_MAXIOSEGMENTS 64 CACHE STRING "max. number of IO segments per group")
set(EC_MAXMBX 1486 CACHE STRING "max. mailbox size")
set(EC_MBXPOOLSIZE 32 CACHE STRING "number of mailboxes in pool")
set(EC_MAXEEPDO 0x200 CACHE STRING "max. eeprom PDO entries")
set(EC_MAXSM 8 CACHE STRING "max. SM used")
set(EC_MAXFMMU 4 CACHE STRING "max. FMMU used")
set(EC_MAXLEN_ADAPTERNAME 128 CACHE STRING "max. adapter name length")
set(EC_MAX_MAPT 1 CACHE STRING "define maximum number of concurrent threads in mapping")
set(EC_MAXODLIST 1024 CACHE STRING "max entries in Object Description list")
set(EC_MAXOELIST 256 CACHE STRING "max entries in Object Entry list")
set(EC_SOE_MAXNAME 60 CACHE STRING "max. length of readable SoE name")
set(EC_SOE_MAXMAPPING 64 CACHE STRING "max. number of SoE mappings")
set(EC_TIMEOUTRET 2000 CACHE STRING "timeout value in us for tx frame to return to rx")
set(EC_TIMEOUTRET3 "EC_TIMEOUTRET * 3" CACHE STRING "timeout value in us for safe data transfer")
set(EC_TIMEOUTSAFE 20000 CACHE STRING "timeout value in us for return safe variant")
set(EC_TIMEOUTEEP 20000 CACHE STRING "timeout value in us for EEPROM access")
set(EC_TIMEOUTTXM 20000 CACHE STRING "timeout value in us for tx mailbox cycle")
set(EC_TIMEOUTRXM 700000 CACHE STRING "timeout value in us for rx mailbox cycle")
set(EC_TIMEOUTSTATE 2000000 CACHE STRING "timeout value in us for check statechange")
set(EC_DEFAULTRETRIES 3 CACHE STRING "default retries if wkc <= 0")
set(EC_PRIMARY_MAC "01:01:01:01:01:01" CACHE STRING "Primary MAC address")
set(EC_SECONDARY_MAC "04:04:04:04:04:04" CACHE STRING "Secondary MAC address")

macro(convert_mac address array)
  set(RE_BYTE "([0-9A-Fa-f][0-9A-Fa-f])")
  string(REGEX REPLACE
    "^${RE_BYTE}:${RE_BYTE}:${RE_BYTE}:${RE_BYTE}:${RE_BYTE}:${RE_BYTE}$"
    "{0x\\1\\2, 0x\\3\\4, 0x\\5\\6}"
    ${array}
    ${address})
endmacro()

convert_mac(${EC_PRIMARY_MAC} EC_PRIMARY_MAC_ARRAY)
convert_mac(${EC_SECONDARY_MAC} EC_SECONDARY_MAC_ARRAY)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/include/soem")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/soem/ec_options.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/soem/ec_options.h"
)

add_library(soem SHARED
  src/ec_base.c
  src/ec_coe.c
  src/ec_config.c
  src/ec_dc.c
  src/ec_eoe.c
  src/ec_foe.c
  src/ec_main.c
  src/ec_print.c
  src/ec_soe.c
  osal/linux/osal.c
  oshw/linux/oshw.c
  oshw/linux/nicdrv.c
)

target_include_directories(soem PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_BINARY_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/osal"
  "${CMAKE_CURRENT_SOURCE_DIR}/osal/linux"
  "${CMAKE_CURRENT_SOURCE_DIR}/oshw/linux"
)

target_link_libraries(soem PUBLIC pthread rt)
