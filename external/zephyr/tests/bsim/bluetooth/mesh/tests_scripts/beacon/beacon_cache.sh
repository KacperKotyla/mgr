#!/usr/bin/env bash
# Copyright 2023 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source $(dirname "${BASH_SOURCE[0]}")/../../_mesh_test.sh

# Test Beacon cache
#
# Test procedure:
# 0. RX device starts monitoring all accepted SNB messages.
# 1. TX device sends two identical SNBs to the RX device.
# 2. TX device sends a secondary SNBs to the RX device, marking the end of the test.
# 3. RX device verifies that only one of the two identical beacons was processed.
RunTest mesh_beacon_cache \
	beacon_tx_beacon_cache \
	beacon_rx_beacon_cache

overlay=overlay_psa_conf
RunTest mesh_beacon_cache \
	beacon_tx_beacon_cache \
	beacon_rx_beacon_cache
