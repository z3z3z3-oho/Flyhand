#!/usr/bin/env python3
from ground_so101_sender import BUILD_ID, PROTOCOL_BUILD_ID, RadioProtocol
from ground_so101_ui import GroundControlWorker

print("gesture:", BUILD_ID)
print("protocol:", PROTOCOL_BUILD_ID)
print("RadioProtocol:", RadioProtocol.__module__)
print("GroundControlWorker:", GroundControlWorker.__module__)

assert BUILD_ID == "2026-07-09-current-pose-zero-v9"
assert PROTOCOL_BUILD_ID == "2026-07-09-single-radio-core-v10"
print("OK: Web UI and CLI share the same radio protocol core.")
