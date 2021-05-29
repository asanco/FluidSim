# because you have more control over the files, but I'm just lazy and will do the recurse glob.
file(GLOB_RECURSE SOURCES
    ./*.hpp
    ./*.cpp
    ./*.h
)

# this creates a library
add_library(Swarm ${SOURCES})

# This set the include paths of the coolcpp library
# We use the public here, because this include path is public to whoever wants to include the library, meaning that
# those include paths will be also included in whoever links this library.
# If the headers were not public, we could set as PRIVATE. We can also do an example on that, if you want.
target_include_directories(Swarm 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)