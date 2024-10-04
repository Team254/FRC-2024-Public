ssh lvuser@10.2.54.2 "ls -t /U/logs/*.wpilog | head -n 5 | tar -cf /U/logs/logdump.tar.gz -T -"
scp lvuser@10.2.54.2:/U/logs/logdump.tar.gz ~
tar -vxf ~/logdump.tar.gz
curl http://10.2.54.11:5807/dumpd > ll_turret_dumpd.log
curl http://10.2.54.11:5807/dump > ll_turret_dump.log
curl http://10.2.54.12:5807/dumpd > ll_elevator_dumpd.log
curl http://10.2.54.12:5807/dump > ll_elevator_dump.log