#!/bin/bash
sleep 2
echo pgheim | sudo -S service robbie stop
sleep 5
echo pgheim | sudo -S shutdown -h now
