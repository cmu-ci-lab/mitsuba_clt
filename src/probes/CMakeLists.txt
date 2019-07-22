if (NOT MTS_VERSION)
    message(FATAL_ERROR "Use the top level configuration file")
endif()

macro(add_probe)
    add_mts_plugin(${ARGN} TYPE probe)
endmacro()

add_probe(columnProbing ColumnProbing.cpp)
add_probe(rowProbing rowProbing.cpp)
add_probe(identityProbing identityProbing.cpp)
add_probe(epipolarProbing epipolarProbing.cpp)