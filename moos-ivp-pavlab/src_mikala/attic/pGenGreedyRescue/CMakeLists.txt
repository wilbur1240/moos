#--------------------------------------------------------
# The CMakeLists.txt for:                        pGenGreedyRescue
# Author(s):                              Mikala Molina
#--------------------------------------------------------

SET(SRC
  GenGreedyRescue.cpp
  GenGreedyRescue_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pGenGreedyRescue ${SRC})

TARGET_LINK_LIBRARIES(pGenGreedyRescue
   contacts 
   geometry
   apputil
   mbutil
   pthread
   ${MOOS_LIBRARIES}
   ${SYSTEM_LIB})
