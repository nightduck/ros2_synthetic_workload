rm callbacks.txt
touch callbacks.txt
echo 'mt_exec,empty_timer' >> callbacks.txt
echo 'mt_exec,publishing_timer' >> callbacks.txt
echo 'mt_exec,high_prio_stage_a_callback' >> callbacks.txt
echo 'mt_exec,med_prio_stage_a_callback' >> callbacks.txt
echo 'mt_exec,low_prio_stage_a_callback' >> callbacks.txt
echo 'mt_exec,high_prio_stage_b_callback' >> callbacks.txt
echo 'mt_exec,med_prio_stage_b_callback' >> callbacks.txt
echo 'mt_exec,low_prio_stage_b_callback' >> callbacks.txt

#objdump -d actor_detection | grep rclcpp | grep Publisher | grep cinematography_msgs | grep msg | grep BoundingBox | grep publish
#objdump -d motion_planner | grep do_intra_process_publish

NEWLINE=$'\n'
set_probes="# Execute as root (sudo alone will not do)
echo > /sys/kernel/debug/tracing/uprobe_events$NEWLINE"
trace_cmd="# Execute as user to collect trace
sudo trace-cmd record "

# Set pointers to all callback functions, used for measuring WCET
while read p; do
    exe=$(echo $p | cut -f1 -d,)

    func=$(echo $p | cut -f2 -d,)
    hex=`objdump -d $exe -C | grep 0000 | grep $func | head -n 1 | cut -f1 -d ' '`

    set_probes+="echo 'p:${exe}_${func}_entry $(pwd)/${exe}:0x$hex' >> /sys/kernel/debug/tracing/uprobe_events$NEWLINE"
    set_probes+="echo 'r:${exe}_${func}_exit $(pwd)/${exe}:0x$hex' >> /sys/kernel/debug/tracing/uprobe_events$NEWLINE"
    trace_cmd+="-e uprobes:${exe}_${func}_entry -e uprobes:${exe}_${func}_exit "
done <callbacks.txt

# Set pointers to all times a node publishes something. Marks release time and used to measure period
while read exe; do 
    while read pubs; do
        pub_hex=$(echo $pubs | cut -f2 -d ' ')
        set_probes+="echo 'r:${exe}_intrapub_${pub_hex} $(pwd)/${exe}:0x$pub_hex' >> /sys/kernel/debug/tracing/uprobe_events$NEWLINE"
        trace_cmd+="-e uprobes:${exe}_intrapub_${pub_hex} "
    done < <(objdump -d $exe | grep do_intra_process_publish | grep callq | cut -f3 | uniq)
    while read pubs; do
        pub_hex=$(echo $pubs | cut -f2 -d ' ')
        set_probes+="echo 'r:${exe}_interpub_${pub_hex} $(pwd)/${exe}:0x${pub_hex}' >> /sys/kernel/debug/tracing/uprobe_events$NEWLINE"
        trace_cmd+="-e uprobes:${exe}_interpub_${pub_hex} "
    done < <(objdump -d $exe | grep do_inter_process_publish | grep callq | cut -f3 | uniq)
done < <(cat callbacks.txt | cut -f1 -d, | uniq)
    
set_probes+="echo 1 > /sys/kernel/debug/tracing/events/uprobes/enable$NEWLINE"

echo "$set_probes"
echo "$trace_cmd"