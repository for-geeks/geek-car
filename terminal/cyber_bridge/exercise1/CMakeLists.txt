cmake_minimum_required (VERSION 2.6)

set (BASE_DIR ${CMAKE_SOURCE_DIR})


project(image_reader)

link_directories(/usr/local/lib)
link_directories(/usr/local/lib/libs_for_shit)

# source files
aux_source_directory (${BASE_DIR} IMG_RDER_SRCS)
aux_source_directory (${BASE_DIR}/../../share_mem SHARE_MEM_SRCS)

# header files

include_directories(${BASE_DIR})
include_directories(${BASE_DIR}/../../share_mem)
include_directories(/usr/local/include/cyber)
include_directories(/usr/local/fast-rtps/include)



# compile flag
set (CMAKE_C_FLAGS "-Wall -g -D_BSD_SOURCE -D_GNU_SOURCE -DDEBUG")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__FILENAME__='\"$(subst ${CMAKE_SOURCE_DIR}/,,$(abspath $<))\"'")
set (CMAKE_CXX_STANDARD 11)

add_executable (${CMAKE_PROJECT_NAME} ${IMG_RDER_SRCS} ${SHARE_MEM_SRCS})
target_link_libraries (${CMAKE_PROJECT_NAME} pthread cyber_core init state glog gflags
blocker_manager protobuf task_manager uuid async_logger
attributes_filler
block
blocker_manager
channel_manager
choreography_conf_proto
choreography_context
classic_conf_proto
classic_context
class_loader
class_loader_manager
component_conf_proto
condition_notifier
croutine
cyber_conf_proto
dag_conf_proto
dispatcher
duration
endpoint
file
global_data
graph
header_builder
identity
intra_dispatcher
intra_message
log_file_object
logger
logger_util
manager
message_info
multicast_notifier
multi_value_warehouse
node
node_manager
notifier_factory
parameter
parameter_client
parameter_proto
parameter_server
participant
participant_listener
perception_proto
perf_event_cache
poller
poll_handler
processor
processor_context
protobuf_factory
proto_desc_proto
py_init
py_node
py_parameter
py_record
py_time
py_timer
py_topology_manager
qos_profile_conf
qos_profile_proto
rate
readable_info
record_file_base
record_file_reader
record_file_writer
record_proto
record_reader
record_viewer
record_writer
role
role_attributes_proto
routine_context
rtps_dispatcher
run_mode_conf_proto
scheduler
scheduler_choreography
scheduler_classic
scheduler_conf_proto
scheduler_factory
segment
service_manager
session
shm_conf
shm_dispatcher
single_value_warehouse
state
sub_listener
subscriber_listener
swap
task_manager
time
timer
timer_component
timing_wheel
topology_change_proto
topology_manager
transport
transport_conf_proto
underlay_message
underlay_message_type
unit_test_proto
)
