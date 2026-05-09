set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)   

idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES freertos_cpp
)

set_source_files_properties(main.cpp PROPERTIES COMPILE_OPTIONS "-save-temps=obj;-fconcepts-diagnostics-depth=2;-fno-devirtualize-speculatively")
add_compile_options("-Werror=return-type")