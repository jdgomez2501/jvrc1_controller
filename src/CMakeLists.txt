set(controller_SRC
  JVRC1Controller.cpp
)

set(controller_HDR
  JVRC1Controller.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DJVRC1Controller_EXPORTS")

