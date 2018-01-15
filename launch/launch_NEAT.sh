#!/bin/bash

# The purpose of this script is to create a launch file for a multi-robot
# system by replicating a "group" tag "n" times, then executing the resulting
# launch file.

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).
n=1

LAUNCH_FILE=/tmp/argos_bridge.launch

echo "<launch>" > $LAUNCH_FILE

for ((i=0; i<n; i++)); do
    namespace="bot$i"
    echo -e "\t<group ns=\"$namespace\">"
    #echo -e "\t\t<node pkg=\"argos_bridge\" type=\"NEAT_controller.py\" name=\"NEAT_controller\" output=\"screen\" />"
    echo -e "\t</group>"
done >> $LAUNCH_FILE

#Launch with gdb
#echo -e "<node pkg=\"argos_bridge\" type=\"argos_ros_start_sim_server\" name=\"argos_ros_start_sim_server\" output=\"screen\" launch-prefix=\"gdb -ex run --args\" />"  >> $LAUNCH_FILE
echo -e "<node pkg=\"argos_bridge\" type=\"argos_ros_start_sim_server\" name=\"argos_ros_start_sim_server\" output=\"screen\">
<param name=\"argos_world_file_name\" value=\"/argos_worlds/simple_environments/no_walls.argos\"/>
<param name=\"file_name_env_path_rel\" value=\"/argos_worlds/experiment_envs/e\"/>
</node>"  >> $LAUNCH_FILE
#echo -e "<node pkg=\"argos_bridge\" type=\"random_environment_generator.py\" name=\"random_environment_generator\" output=\"screen\"/>"  >> $LAUNCH_FILE


echo -e "<node pkg=\"neat_ros\" type=\"neat_ros_node\" name=\"neat_ros_node\" output=\"screen\" >"  >> $LAUNCH_FILE
echo -e "<param name=\"input_file\" value=\"/neat_params.ne\" />"  >> $LAUNCH_FILE
#echo -e "<param name=\"input_file\" value=\"/overall_winner\" />"  >> $LAUNCH_FILE
#echo -e "<param name=\"input_file\" value=\"/archive_genomes/g18\" />"  >> $LAUNCH_FILE
echo -e "</node>"  >> $LAUNCH_FILE

echo -e "</launch>" >> $LAUNCH_FILE

roslaunch $LAUNCH_FILE
