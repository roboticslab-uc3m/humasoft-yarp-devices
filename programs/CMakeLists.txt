# Copyright: Universidad Carlos III de Madrid (C) 2020
# Authors: Raúl de Santos Rico

option(ENABLE_programs "Enable/disable demostration programs" OFF)

if(ENABLE_programs)
	add_subdirectory(stepInputDemo)
	add_subdirectory(joystickInputDemo)
	add_subdirectory(tightenTendons)
endif()
