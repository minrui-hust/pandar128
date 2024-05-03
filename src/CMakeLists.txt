add_library(pandar128_driver driver.cc input.cc laser_ts.cc platUtil.cc util.c
                             tcp_command_client.c)
target_link_libraries(pandar128_driver pcap ${OPENSSL_LIBRARIES}
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
