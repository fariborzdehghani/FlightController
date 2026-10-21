#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
mkdir -p build/host-tests
cc -std=c11 -Wall -Wextra -Werror -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Itests/host -ICore/Inc -ICore/Modules/Inc \
  tests/host/takeoff_test.c Core/Src/FlightControl.c Core/Src/PID.c \
  Core/Modules/Src/CoreHelpers.c Core/Modules/Src/AppError.c \
  -lm -o build/host-tests/takeoff_test
build/host-tests/takeoff_test
cc -std=c11 -Wall -Wextra -Werror -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Itests/host -ICore/Inc \
  tests/host/srf05_test.c Core/Src/srf05.c \
  -lm -o build/host-tests/srf05_test
build/host-tests/srf05_test
cc -std=c11 -Wall -Wextra -Werror -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -ICore/Inc -ICore/Modules/Inc \
  tests/host/srf05_filter_test.c Core/Modules/Src/VelocityEstimator.c \
  -lm -o build/host-tests/srf05_filter_test
build/host-tests/srf05_filter_test
cc -std=c11 -Wall -Wextra -Werror -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -ICore/Inc tests/host/flight_recorder_test.c Core/Src/FlightRecorder.c \
  -lm -o build/host-tests/flight_recorder_test
build/host-tests/flight_recorder_test
cc -std=c11 -Wall -Wextra -Werror -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -ICore/Inc tests/host/diagnostics_test.c Core/Src/FlightRecorder.c \
  -lm -o build/host-tests/diagnostics_test
build/host-tests/diagnostics_test
cc -std=c11 -Wall -Wextra -Werror -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -ICore/Inc tests/host/compact_flight_log_test.c Core/Src/FlightRecorder.c \
  -lm -o build/host-tests/compact_flight_log_test
build/host-tests/compact_flight_log_test
for headers in Core/Inc ../RemoteController/Core/Inc; do
  cc -std=c11 -Wall -Wextra -Werror -I"$headers" \
    tests/host/flight_log_protocol_test.c -o build/host-tests/flight_log_protocol_test
  build/host-tests/flight_log_protocol_test
done
