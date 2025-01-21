#--------------------------------------------------------
# The CMakeLists.txt for:                       pSendPoints
# Author(s):                              Mikala Molina
#--------------------------------------------------------

SET(SRC
  SendPoints.cpp
  SendPoints_Info.cpp
  VisitPoint.cpp
  NodeReport.cpp
  main.cpp
)

ADD_EXECUTABLE(pSendPoints ${SRC})

TARGET_LINK_LIBRARIES(pSendPoints
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)

