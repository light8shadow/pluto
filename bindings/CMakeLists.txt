add_subdirectory(csharp)
add_subdirectory(python)

option(CPP_BINDINGS "Install C++11 bindings" ON)
if (CPP_BINDINGS)
	add_subdirectory(cpp)
endif()
